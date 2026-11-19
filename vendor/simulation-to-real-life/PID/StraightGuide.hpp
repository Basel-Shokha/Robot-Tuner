#pragma once
#include "Types.hpp"
#include "Raycast.hpp"
#include <vector>
#include <algorithm>
#include <cmath>

// Ground-truth reference for a STRAIGHT track, used only by the trainer's reward (the robot never
// sees it). Answers two questions at any pose: "how far is the heading from the track direction"
// and "how far is the robot from the corridor centre".
//
// Both answers must ignore wall errors (slanted segments, bumps, gaps, per-episode jitter), so:
//  - The track AXIS is one global direction: the length-weighted axial mean of every wall's
//    direction. A single slanted segment barely moves it, so "parallel" means parallel to the
//    track, not to whichever crooked piece of wall is nearby.
//  - The CENTRE at longitudinal position s is the midpoint of the MEDIAN lateral position of each
//    wall over a window of +-window_m around s. A median ignores a minority of crooked samples,
//    but still follows genuine width changes (widening corridors, sections of different width).
//
// Built once from the UNPERTURBED walls, so the per-episode wall jitter is an error to ignore too.
struct StraightGuide {
    static constexpr float BIN_M = 0.05f;       // longitudinal resolution of the precomputed centre
    static constexpr float SAMPLE_M = 0.01f;    // wall sampling step

    bool valid = false;
    Vec2 origin{0.0f, 0.0f};  // spawn point; s and lateral are measured from here
    Vec2 u{1.0f, 0.0f};       // unit track direction (spawn -> finish)
    Vec2 n{0.0f, 1.0f};       // unit left normal
    float axis_angle = 0.0f;  // atan2(u.y, u.x), radians
    float s0 = 0.0f;          // s of bin 0
    std::vector<float> center_lat;  // per bin: corridor centre (lateral), NaN when unknown
    std::vector<float> half_width;  // per bin: half the wall-to-wall distance, NaN when unknown

    struct Query {
        bool ok = false;
        float s = 0.0f;           // distance along the track axis from spawn
        float lateral = 0.0f;     // signed distance from the axis line through spawn (+ = left)
        float center_err = 0.0f;  // lateral - centre, metres
        float half_width = 0.0f;  // local robust half width, metres
        float heading_err = 0.0f; // robot heading minus track direction, wrapped to [-pi, pi]
        float tangent = 0.0f;     // local track direction (rad): the axis, or the centreline tangent
    };

    // Centreline mode (curved and narrow tracks, tools/gen_narrow_tracks.py): the track file lists the
    // nominal centreline as "center x y half_width" lines. Then s = arc length of the nearest centreline
    // point, the centre error is the signed distance from it (+ = left), the heading error is measured
    // against the local tangent, and half_width is the nominal one (wall errors excluded by construction).
    bool poly = false;
    std::vector<Vec2> cpts;
    std::vector<float> chw, cs;  // half width and arc length at each centreline point

    void BuildPolyline(const std::vector<Vec2>& pts, const std::vector<float>& hw) {
        if (pts.size() < 2) return;
        cpts = pts; chw = hw; cs.assign(pts.size(), 0.0f);
        for (size_t i = 1; i < pts.size(); ++i) cs[i] = cs[i - 1] + std::hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
        poly = true; valid = true;
        origin = pts[0];
        axis_angle = std::atan2(pts[1].y - pts[0].y, pts[1].x - pts[0].x);
        u = {std::cos(axis_angle), std::sin(axis_angle)};
        n = {-u.y, u.x};
    }

    Query AtPolyline(Vec2 pos, float theta) const {
        Query q;
        float best = 1e30f;
        for (size_t i = 0; i + 1 < cpts.size(); ++i) {
            Vec2 a = cpts[i], d = cpts[i + 1] - cpts[i];
            float L2 = d.x * d.x + d.y * d.y;
            if (L2 < 1e-12f) continue;
            float t = std::clamp(((pos.x - a.x) * d.x + (pos.y - a.y) * d.y) / L2, 0.0f, 1.0f);
            float px = a.x + d.x * t - pos.x, py = a.y + d.y * t - pos.y;
            float dist2 = px * px + py * py;
            if (dist2 < best) {
                best = dist2;
                float L = std::sqrt(L2);
                q.s = cs[i] + t * L;
                q.tangent = std::atan2(d.y, d.x);
                q.center_err = ((pos.x - a.x) * (-d.y) + (pos.y - a.y) * d.x) / L;  // + = left of the centreline
                q.half_width = chw[i] + (chw[i + 1] - chw[i]) * t;
            }
        }
        q.lateral = q.center_err;
        q.heading_err = WrapPi(theta - q.tangent);
        q.ok = best < 1e29f;
        return q;
    }

    static float WrapPi(float a) {
        while (a > PI_F) a -= 2.0f * PI_F;
        while (a < -PI_F) a += 2.0f * PI_F;
        return a;
    }

    static float Median(std::vector<float>& v) {
        size_t m = v.size() / 2;
        std::nth_element(v.begin(), v.begin() + m, v.end());
        return v[m];
    }

    // forward_hint: a point the track leads toward (finish-line midpoint); used only for the sign
    // of the axis. has_hint=false falls back to the wall-length centroid.
    void Build(const std::vector<Wall>& walls, Vec2 spawn, bool has_hint, Vec2 forward_hint, float window_m = 0.6f) {
        valid = false;
        if (walls.empty()) return;

        // Axial mean of wall directions: doubling the angle makes a wall and its reverse agree.
        float c2 = 0.0f, s2 = 0.0f, total_len = 0.0f;
        Vec2 centroid{0.0f, 0.0f};
        for (const auto& w : walls) {
            float dx = w.p2.x - w.p1.x, dy = w.p2.y - w.p1.y;
            float len = std::sqrt(dx * dx + dy * dy);
            if (len < 1e-6f) continue;
            float a = std::atan2(dy, dx);
            c2 += len * std::cos(2.0f * a);
            s2 += len * std::sin(2.0f * a);
            centroid = centroid + (w.p1 + w.p2) * (0.5f * len);
            total_len += len;
        }
        if (total_len <= 0.0f) return;
        centroid = centroid * (1.0f / total_len);

        float a = 0.5f * std::atan2(s2, c2);
        u = {std::cos(a), std::sin(a)};
        Vec2 toward = (has_hint ? forward_hint : centroid) - spawn;
        if (u.x * toward.x + u.y * toward.y < 0.0f) u = u * -1.0f;
        n = {-u.y, u.x};
        axis_angle = std::atan2(u.y, u.x);
        origin = spawn;

        // Sample the walls that run along the track; end caps and width steps are skipped.
        std::vector<std::pair<float, float>> samples;  // (s, lateral)
        float s_min = 1e9f, s_max = -1e9f;
        for (const auto& w : walls) {
            Vec2 d = w.p2 - w.p1;
            float len = std::sqrt(d.x * d.x + d.y * d.y);
            if (len < 1e-6f) continue;
            if (std::abs((d.x * u.x + d.y * u.y) / len) < 0.7f) continue;
            int k = std::max(1, (int)std::ceil(len / SAMPLE_M));
            for (int i = 0; i <= k; ++i) {
                Vec2 p = w.p1 + d * ((float)i / (float)k);
                Vec2 r = p - origin;
                float s = r.x * u.x + r.y * u.y;
                float lat = r.x * n.x + r.y * n.y;
                samples.push_back({s, lat});
                s_min = std::min(s_min, s);
                s_max = std::max(s_max, s);
            }
        }
        if (samples.size() < 4) return;
        std::sort(samples.begin(), samples.end());

        s0 = s_min;
        int bins = (int)std::ceil((s_max - s_min) / BIN_M) + 1;
        center_lat.assign(bins, NAN);
        half_width.assign(bins, NAN);

        std::vector<float> window, left, right;
        for (int b = 0; b < bins; ++b) {
            float sc = s0 + b * BIN_M;
            auto lo = std::lower_bound(samples.begin(), samples.end(), std::make_pair(sc - window_m, -1e9f));
            auto hi = std::upper_bound(samples.begin(), samples.end(), std::make_pair(sc + window_m, 1e9f));
            window.clear();
            for (auto it = lo; it != hi; ++it) window.push_back(it->second);
            if (window.size() < 4) continue;

            // The two walls are the two clusters either side of the largest lateral gap.
            std::sort(window.begin(), window.end());
            size_t split = 0;
            float best_gap = 0.0f;
            for (size_t i = 1; i < window.size(); ++i) {
                float g = window[i] - window[i - 1];
                if (g > best_gap) { best_gap = g; split = i; }
            }
            if (best_gap < 0.10f) continue;  // only one wall in view (e.g. past the corridor end)

            right.assign(window.begin(), window.begin() + split);
            left.assign(window.begin() + split, window.end());
            float r_med = Median(right);
            float l_med = Median(left);
            center_lat[b] = 0.5f * (l_med + r_med);
            half_width[b] = 0.5f * (l_med - r_med);
        }

        // Fill unknown bins from the nearest known one, so a query never lands in a hole.
        int last = -1;
        for (int b = 0; b < bins; ++b) {
            if (!std::isnan(center_lat[b])) { last = b; continue; }
            if (last >= 0) { center_lat[b] = center_lat[last]; half_width[b] = half_width[last]; }
        }
        last = -1;
        for (int b = bins - 1; b >= 0; --b) {
            if (!std::isnan(center_lat[b])) { last = b; continue; }
            if (last >= 0) { center_lat[b] = center_lat[last]; half_width[b] = half_width[last]; }
        }
        valid = last >= 0 || !std::isnan(center_lat[0]);
    }

    Query At(Vec2 pos, float theta) const {
        if (poly) return AtPolyline(pos, theta);
        Query q;
        if (!valid) return q;
        Vec2 r = pos - origin;
        q.s = r.x * u.x + r.y * u.y;
        q.lateral = r.x * n.x + r.y * n.y;
        int b = (int)std::lround((q.s - s0) / BIN_M);
        b = std::clamp(b, 0, (int)center_lat.size() - 1);
        if (std::isnan(center_lat[b])) return q;
        q.center_err = q.lateral - center_lat[b];
        q.half_width = half_width[b];
        q.heading_err = WrapPi(theta - axis_angle);
        q.tangent = axis_angle;
        q.ok = true;
        return q;
    }
};
