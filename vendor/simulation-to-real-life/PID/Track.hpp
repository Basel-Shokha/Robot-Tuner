#pragma once
#include "Types.hpp"
#include "Raycast.hpp"
#include "StraightGuide.hpp"
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>

// Optional finish segment; the robot completes the track when its chassis crosses it.
struct FinishLine {
    bool has_finish = false;
    Wall segment{};
};

// Per-run wall jitter (wall_noise_std_m). Connected walls keep sharing their endpoints, and along a chain of
// connected walls the offset is interpolated between random knots every KNOT_M of wall length, so a curved wall
// made of 5 cm pieces stays a smooth (slightly displaced, slightly wavy) board instead of a 10 mm sawtooth. A
// single long wall (the older straight tracks) still gets an independent offset at each end, as before.
inline std::vector<Wall> PerturbTrack(const std::vector<Wall>& track, float noise_std, std::mt19937& rng) {
    if (noise_std <= 0.0f) return track;
    const float KNOT_M = 0.30f, SAME = 1e-4f;
    std::normal_distribution<float> dist(0.0f, noise_std);
    // Unique vertices and which walls use them.
    std::vector<Vec2> V;
    std::vector<int> wv(2 * track.size());
    auto vid = [&](Vec2 p) {
        for (size_t i = 0; i < V.size(); ++i)
            if (std::abs(V[i].x - p.x) < SAME && std::abs(V[i].y - p.y) < SAME) return (int)i;
        V.push_back(p);
        return (int)V.size() - 1;
    };
    for (size_t i = 0; i < track.size(); ++i) { wv[2 * i] = vid(track[i].p1); wv[2 * i + 1] = vid(track[i].p2); }
    std::vector<std::vector<int>> adj(V.size());  // wall indices per vertex
    for (size_t i = 0; i < track.size(); ++i) { adj[wv[2 * i]].push_back((int)i); adj[wv[2 * i + 1]].push_back((int)i); }
    std::vector<Vec2> off(V.size());
    std::vector<char> vdone(V.size(), 0), wdone(track.size(), 0);
    auto walk = [&](int start) {  // follow unvisited walls from `start`, assigning interpolated offsets
        Vec2 k0{dist(rng), dist(rng)}, k1{dist(rng), dist(rng)};
        float s = 0.0f, knot_s = 0.0f;
        if (!vdone[start]) { off[start] = k0; vdone[start] = 1; } else { k0 = off[start]; }
        int v = start;
        for (;;) {
            int w = -1;
            for (int c : adj[v]) if (!wdone[c]) { w = c; break; }
            if (w < 0) return;
            wdone[w] = 1;
            int u = wv[2 * w] == v ? wv[2 * w + 1] : wv[2 * w];
            s += std::hypot(V[u].x - V[v].x, V[u].y - V[v].y);
            while (s > knot_s + KNOT_M) { knot_s += KNOT_M; k0 = k1; k1 = {dist(rng), dist(rng)}; }
            float f = std::clamp((s - knot_s) / KNOT_M, 0.0f, 1.0f);
            if (!vdone[u]) { off[u] = {k0.x + (k1.x - k0.x) * f, k0.y + (k1.y - k0.y) * f}; vdone[u] = 1; }
            v = u;
        }
    };
    for (size_t i = 0; i < V.size(); ++i) if (adj[i].size() == 1 && !vdone[i]) walk((int)i);  // open chains first
    for (size_t i = 0; i < V.size(); ++i) if (!vdone[i]) walk((int)i);                       // closed loops
    std::vector<Wall> perturbed = track;
    for (size_t i = 0; i < track.size(); ++i) {
        perturbed[i].p1 = track[i].p1 + off[wv[2 * i]];
        perturbed[i].p2 = track[i].p2 + off[wv[2 * i + 1]];
    }
    return perturbed;
}

inline std::vector<Wall> GenerateBenchmarkTrack(Vec2& out_spawn_point, float& out_spawn_heading_rad) {
    out_spawn_point = {2.5f, 1.6f};
    out_spawn_heading_rad = 0.0f;
    return {
        {{1.0f, 1.0f}, {8.0f, 1.0f}},
        {{8.0f, 1.0f}, {9.5f, 2.5f}},
        {{9.5f, 2.5f}, {8.5f, 4.0f}},
        {{8.5f, 4.0f}, {9.8f, 5.5f}},
        {{9.8f, 5.5f}, {8.5f, 8.0f}},
        {{8.5f, 8.0f}, {4.0f, 8.5f}},
        {{4.0f, 8.5f}, {1.0f, 6.5f}},
        {{1.0f, 6.5f}, {1.0f, 1.0f}},
        {{2.2f, 2.2f}, {6.8f, 2.2f}},
        {{6.8f, 2.2f}, {7.8f, 3.0f}},
        {{7.8f, 3.0f}, {7.0f, 4.2f}},
        {{7.0f, 4.2f}, {8.0f, 5.4f}},
        {{8.0f, 5.4f}, {7.0f, 6.8f}},
        {{7.0f, 6.8f}, {3.8f, 7.2f}},
        {{3.8f, 7.2f}, {2.2f, 5.5f}},
        {{2.2f, 5.5f}, {2.2f, 2.2f}}
    };
}

inline std::vector<Wall> LoadTrackFromFile(const std::string& filepath, Vec2& out_spawn_point, float& out_spawn_heading_rad,
                                           FinishLine* out_finish = nullptr) {
    if (out_finish) *out_finish = FinishLine{};
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cout << "Error: Could not open track file " << filepath << "\n";
        return GenerateBenchmarkTrack(out_spawn_point, out_spawn_heading_rad);
    }

    std::vector<std::string> lines;
    std::string l;
    bool is_freeform = false;

    while (std::getline(file, l)) {
        if (!l.empty() && l.back() == '\r') l.pop_back();
        if (l.find("wall ") == 0 || l.find("spawn ") == 0 || l.find("finish ") == 0) {
            is_freeform = true;
        }
        lines.push_back(l);
    }

    if (is_freeform) {
        out_spawn_point = {0.0f, 0.0f};
        out_spawn_heading_rad = 0.0f;
        std::vector<Wall> walls;

        for (const auto& line : lines) {
            if (line.empty() || line[0] == '#') continue;
            std::stringstream ss(line);
            std::string tag;
            ss >> tag;
            if (tag == "spawn") {
                float heading_deg = 0.0f;
                ss >> out_spawn_point.x >> out_spawn_point.y >> heading_deg;
                out_spawn_heading_rad = heading_deg * (PI_F / 180.0f);
            } else if (tag == "wall") {
                Wall w;
                ss >> w.p1.x >> w.p1.y >> w.p2.x >> w.p2.y;
                walls.push_back(w);
            } else if (tag == "finish" && out_finish) {
                Wall f;
                if (ss >> f.p1.x >> f.p1.y >> f.p2.x >> f.p2.y) {
                    out_finish->has_finish = true;
                    out_finish->segment = f;
                }
            }
        }
        return walls;
    }

    float corridor_width = 0.30f;
    float segment_length = 0.25f;
    float heading_deg = 0.0f;
    std::vector<Vec2> centerline;
    centerline.push_back({0.0f, 0.0f});
    Vec2 curr = {0.0f, 0.0f};
    bool in_segments = false;

    for (const auto& line : lines) {
        if (line.empty() || line[0] == '#') continue;
        std::stringstream ss(line);
        std::string token;
        if (!(ss >> token)) continue;

        if (token == "segments:") {
            in_segments = true;
            continue;
        }

        if (!in_segments) {
            if (token == "corridor_width") ss >> corridor_width;
            else if (token == "segment_length") ss >> segment_length;
            else if (token == "start_heading") ss >> heading_deg;
        } else {
            float turn_deg = std::stof(token);
            heading_deg += turn_deg;
            float rad = heading_deg * PI_F / 180.0f;
            curr.x += std::cos(rad) * segment_length;
            curr.y += std::sin(rad) * segment_length;
            centerline.push_back(curr);
        }
    }

    if (centerline.size() > 1) {
        out_spawn_point = {
            (centerline[0].x + centerline[1].x) * 0.5f,
            (centerline[0].y + centerline[1].y) * 0.5f
        };
        float dx = centerline[1].x - centerline[0].x;
        float dy = centerline[1].y - centerline[0].y;
        out_spawn_heading_rad = std::atan2(dy, dx);
    } else {
        out_spawn_point = centerline[0];
        out_spawn_heading_rad = 0.0f;
    }

    std::vector<Wall> walls;
    float w2 = corridor_width / 2.0f;
    for (size_t i = 0; i < centerline.size() - 1; ++i) {
        Vec2 p1 = centerline[i];
        Vec2 p2 = centerline[i + 1];
        float dx = p2.x - p1.x;
        float dy = p2.y - p1.y;
        float len = std::sqrt(dx * dx + dy * dy);
        if (len < 1e-5f) continue;
        Vec2 perp = {-dy / len, dx / len};
        walls.push_back({p1 + perp * w2, p2 + perp * w2});
        walls.push_back({p1 - perp * w2, p2 - perp * w2});
    }
    return walls;
}

// Track files live in ONE shared folder at the repository root (tracks/). Run from a controller folder
// (NN/, PID/, LineFit/) that is ../tracks/; a work directory with its own tracks/ copy is used first.
inline std::string TrackPath(const std::string& name) {
    std::string local = "tracks/" + name + ".track";
    if (std::ifstream(local).good()) return local;
    return "../tracks/" + name + ".track";
}

inline std::vector<Wall> GenerateTrack(const std::string& track_name, Vec2& out_spawn_point, float& out_spawn_heading_rad,
                                       FinishLine* out_finish = nullptr) {
    if (out_finish) *out_finish = FinishLine{};
    if (track_name == "benchmark") return GenerateBenchmarkTrack(out_spawn_point, out_spawn_heading_rad);

    std::string filepath = TrackPath(track_name);
    std::ifstream f(filepath);
    if (f.good()) return LoadTrackFromFile(filepath, out_spawn_point, out_spawn_heading_rad, out_finish);

    std::cout << "Warning: Unknown track '" << track_name << "', defaulting to benchmark.\n";
    return GenerateBenchmarkTrack(out_spawn_point, out_spawn_heading_rad);
}

// A fully loaded track, so multi-track training can load everything once at startup.
struct LoadedTrack {
    std::string name;
    std::vector<Wall> walls;
    Vec2 spawn{0.0f, 0.0f};
    float spawn_heading = 0.0f;
    FinishLine finish;
    StraightGuide guide;  // ground-truth axis + robust centre, for the straight-track reward
};

// Optional nominal centreline of a track file: "center x y half_width" lines (tools/gen_narrow_tracks.py).
inline bool LoadCenterline(const std::string& filepath, std::vector<Vec2>& pts, std::vector<float>& hw) {
    std::ifstream f(filepath);
    std::string line;
    while (std::getline(f, line)) {
        std::stringstream ss(line);
        std::string tag;
        if (!(ss >> tag) || tag != "center") continue;
        Vec2 p; float h;
        if (ss >> p.x >> p.y >> h) { pts.push_back(p); hw.push_back(h); }
    }
    return pts.size() >= 2;
}

inline LoadedTrack LoadNamedTrack(const std::string& name) {
    LoadedTrack t;
    t.name = name;
    t.walls = GenerateTrack(name, t.spawn, t.spawn_heading, &t.finish);
    std::vector<Vec2> cp; std::vector<float> hw;
    if (LoadCenterline(TrackPath(name), cp, hw)) {
        t.guide.BuildPolyline(cp, hw);  // curved / narrow tracks: reward along the nominal centreline
    } else {
        Vec2 finish_mid = (t.finish.segment.p1 + t.finish.segment.p2) * 0.5f;
        t.guide.Build(t.walls, t.spawn, t.finish.has_finish, finish_mid);
    }
    return t;
}

inline std::vector<LoadedTrack> LoadTrackSet(const std::vector<std::string>& names) {
    std::vector<LoadedTrack> out;
    for (const auto& n : names) out.push_back(LoadNamedTrack(n));
    return out;
}