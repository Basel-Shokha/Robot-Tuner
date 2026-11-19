#pragma once
#include "Types.hpp"
#include "SimConfig.hpp"
#include <vector>
#include <cmath>
#include <random>
#include <algorithm>

constexpr float PI_F = 3.14159265359f;
// ToF cone half-angle, horizontal, in radians.
//
// The VL53L1X datasheet (DocID031281 Rev 2 / DS11054) Table 1 and Table 9 publish the FoV as a
// DIAGONAL figure tied to the SPAD ROI: 16x16 -> 27 deg, 8x8 -> 20 deg, 4x4 -> 15 deg. The ULD
// driver leaves the ROI at the full 16x16 array unless VL53L1X_SetROI() is called, and no ROI
// programming exists anywhere in this repo or in weights.h, so 16x16 / 27 deg diagonal is the
// operating point assumed here. Re-derive cfg.tof_cone_half_angle_rad from the table if the firmware
// ever narrows the ROI -- a 4x4 ROI is barely half this cone.
//
// This simulation scans in a plane, so what matters is the HORIZONTAL FoV, not the diagonal.
// For a square ROI under a near-rectilinear lens mapping:
//     tan(fov_h / 2) = tan(fov_d / 2) / sqrt(2)
//     fov_h = 2 * atan(tan(13.5 deg) / sqrt(2)) = 19.27 deg  ->  half-angle 0.16816 rad
// (The cruder small-angle form fov_h = fov_d / sqrt(2) = 19.09 deg gives 0.16661 rad. The two
// differ by 1%, far inside the sensor's own tolerance; the exact form is used.)
//
// The previous value, 0.218 rad, corresponds to a 24.98 deg FULL horizontal cone -- it treated the
// published diagonal figure as if it were the horizontal one. The cone was ~30% too wide.
// Now runtime-configurable: tof_fov_diagonal_deg / tof_fov_horizontal_deg in error_params.txt, turned
// into cfg.tof_cone_half_angle_rad by LoadConfig (default 27 deg diagonal -> 0.16816 rad, as above).

constexpr int TOF_MAX_RAYS = 33;

// A single ray's intersection with the track. cos_incidence is |cos| of the angle between the ray
// and the surface normal: 1.0 head-on, 0.0 grazing. Meaningless when hit is false.
struct RayHit {
    float dist = 0.0f;
    float cos_incidence = 0.0f;
    bool hit = false;
};

inline RayHit CastSingleRayHit(Vec2 origin, float angle, const std::vector<Wall>& walls, float tof_max_range) {
    float rdx = std::cos(angle);
    float rdy = std::sin(angle);
    RayHit best;
    best.dist = tof_max_range;

    for (const auto& w : walls) {
        float v1x = origin.x - w.p1.x;
        float v1y = origin.y - w.p1.y;
        float v2x = w.p2.x - w.p1.x;
        float v2y = w.p2.y - w.p1.y;

        float cross = v2x * (-rdy) - v2y * (-rdx);
        if (std::abs(cross) < 1e-6f) continue;

        float t1 = (v2x * v1y - v2y * v1x) / cross;
        float t2 = (v1x * (-rdy) - v1y * (-rdx)) / cross;

        if (t1 >= 0.0f && t2 >= 0.0f && t2 <= 1.0f) {
            if (t1 < best.dist) {
                best.dist = t1;
                best.hit = true;
                // The wall normal is the segment direction rotated 90 deg. Only |cos| is used, so
                // which of the two normals this picks does not matter.
                float len = std::sqrt(v2x * v2x + v2y * v2y);
                if (len > 1e-9f) {
                    float nx = -v2y / len;
                    float ny =  v2x / len;
                    best.cos_incidence = std::abs(rdx * nx + rdy * ny);
                } else {
                    best.cos_incidence = 1.0f;
                }
            }
        }
    }
    return best;
}

// Distance-only form, for callers that do not need the surface normal.
inline float CastSingleRay(Vec2 origin, float angle, const std::vector<Wall>& walls, float tof_max_range) {
    return CastSingleRayHit(origin, angle, walls, tof_max_range).dist;
}

// Fills out_angles with `rays` offsets evenly spanning [-half_angle, +half_angle].
// A single ray degenerates to the boresight. Returns the number written.
inline int BuildConeOffsets(int rays, float half_angle, float* out_angles, int max_rays) {
    int n = std::clamp(rays, 1, max_rays);
    if (n == 1) {
        out_angles[0] = 0.0f;
        return 1;
    }
    for (int i = 0; i < n; ++i) {
        float t = (float)i / (float)(n - 1);       // 0 .. 1
        out_angles[i] = (t * 2.0f - 1.0f) * half_angle;
    }
    return n;
}

// ---------------------------------------------------------------------------------------------
// Model A (cheap, historical): nearest surface in the cone.
//
// Reports min() over the cone rays. This is not what a VL53L1X does -- the sensor has no way to
// single out the nearest surface in its FoV -- but it is one raycast per ray with no normals and
// no sorting, so it is the cheap option. Kept as the default; see tof_signal_model.
// ---------------------------------------------------------------------------------------------
inline float SampleToFMinOverCone(Vec2 origin, float world_angle, const std::vector<Wall>& walls,
                                  const SimConfig& cfg) {
    float offsets[TOF_MAX_RAYS];
    int n = BuildConeOffsets(cfg.tof_rays, cfg.tof_cone_half_angle_rad, offsets, TOF_MAX_RAYS);

    float closest = cfg.tof_max_range_m;
    for (int i = 0; i < n; ++i) {
        float d = CastSingleRay(origin, world_angle + offsets[i], walls, cfg.tof_max_range_m);
        if (d < closest) closest = d;
    }
    return closest;
}

// ---------------------------------------------------------------------------------------------
// Model B: histogram / returned-signal model.
//
// A VL53L1X is a SPAD array feeding a histogram dToF engine. It does not report the nearest
// surface in the cone; it reports the distance of the histogram peak with the strongest returned
// signal, and rejects the measurement outright when that peak is too weak. Three behaviours
// follow that min() cannot reproduce:
//
//   - grazing surfaces return too little light and drop out, rather than reading short;
//   - an off-axis object is lit by fewer SPADs than an on-axis one, so the cone centre dominates;
//   - a corner puts two competing peaks in the histogram, and the reported range lands between
//     them rather than resolving the near edge sharply.
//
// Per ray: signal ~ reflectance * cos^k(incidence) * offaxis_weight / d^2.
//
// CALIBRATION of tof_signal_gain_kcps_m2 (K). The per-ray off-axis weights are normalised to sum
// to 1, so a flat wall square-on at distance d returns exactly K * reflectance / d^2 kcps. The
// datasheet anchor is the max-range condition: an 88% white target in the dark, long distance
// mode, timing budget 100 ms, reaches 360 cm (Table 6), at which point the return has fallen to
// the ULD's default signal threshold of 1024 kcps. So
//     K = 1024 * 3.60^2 / 0.88 = 15080 kcps*m^2
// Checked against the other rows of Table 6, which were NOT used to fit it:
//     grey 17%: sqrt(K * 0.17 / 1024) = 1.58 m   (datasheet 1.70 m, -7%)
//     grey 54%: sqrt(K * 0.54 / 1024) = 2.82 m   (datasheet 3.40 m, -17%)
// A plain inverse-square law with one fitted constant lands within 7-17% of the published range
// for reflectances it never saw, which is about as good as this model deserves to be.
// ---------------------------------------------------------------------------------------------
inline float SampleToFSignalModel(Vec2 origin, float world_angle, const std::vector<Wall>& walls,
                                  const SimConfig& cfg, std::mt19937& rng) {
    float offsets[TOF_MAX_RAYS];
    int n = BuildConeOffsets(cfg.tof_rays, cfg.tof_cone_half_angle_rad, offsets, TOF_MAX_RAYS);

    // Off-axis weight: a Gaussian in the ray's angle off boresight, with a standard deviation set
    // as a fraction of the cone half-angle. This stands in for the illumination and SPAD-count
    // falloff toward the edge of the ROI, which the datasheet does not quantify. Normalising the
    // weights to sum to 1 is what makes the calibration of K above independent of the ray count.
    float sigma_off = std::max(1e-4f, cfg.tof_offaxis_sigma_frac * cfg.tof_cone_half_angle_rad);
    float weights[TOF_MAX_RAYS];
    float weight_sum = 0.0f;
    for (int i = 0; i < n; ++i) {
        float a = offsets[i] / sigma_off;
        weights[i] = std::exp(-0.5f * a * a);
        weight_sum += weights[i];
    }

    float dists[TOF_MAX_RAYS];
    float signals[TOF_MAX_RAYS];
    int hits = 0;

    for (int i = 0; i < n; ++i) {
        RayHit h = CastSingleRayHit(origin, world_angle + offsets[i], walls, cfg.tof_max_range_m);
        if (!h.hit || h.dist < 1e-4f) continue;

        float cos_term = std::pow(std::max(0.0f, h.cos_incidence), cfg.tof_cos_power);
        float s = cfg.tof_signal_gain_kcps_m2 * cfg.tof_wall_reflectance * cos_term
                * (weights[i] / weight_sum) / (h.dist * h.dist);
        if (s <= 0.0f) continue;

        dists[hits] = h.dist;
        signals[hits] = s;
        ++hits;
    }

    // Nothing in the cone returned any light at all.
    if (hits == 0) return cfg.tof_max_range_m;

    // Sort rays by distance so histogram peaks are contiguous runs.
    int order[TOF_MAX_RAYS];
    for (int i = 0; i < hits; ++i) order[i] = i;
    std::sort(order, order + hits, [&](int a, int b) { return dists[a] < dists[b]; });

    // Group into peaks: returns closer together than the histogram bin width merge into one peak.
    // The strongest peak wins, and its reported distance is the signal-weighted centroid of its
    // rays -- which is why a corner rounds off instead of resolving its near edge.
    float best_signal = -1.0f;
    float best_dist = cfg.tof_max_range_m;

    int g = 0;
    while (g < hits) {
        int end = g + 1;
        while (end < hits && (dists[order[end]] - dists[order[end - 1]]) < cfg.tof_peak_merge_m) ++end;

        float sum_s = 0.0f, sum_sd = 0.0f;
        for (int i = g; i < end; ++i) {
            sum_s  += signals[order[i]];
            sum_sd += signals[order[i]] * dists[order[i]];
        }
        if (sum_s > best_signal) {
            best_signal = sum_s;
            best_dist = sum_sd / sum_s;
        }
        g = end;
    }

    // Reject the measurement when the winning peak falls below the driver's signal threshold. The
    // ULD reports range status 2 here and the firmware gets no distance, which this sim represents
    // the same way it represents a dropout -- as max range.
    if (best_signal < cfg.tof_signal_threshold_kcps) return cfg.tof_max_range_m;

    // Ranging noise. The baseline is the same term the min() path uses, so that switching models
    // changes the geometry and the peak selection without also changing the noise floor. On top of
    // that, sigma grows as the return weakens (photon shot noise, sigma ~ 1/sqrt(signal)), applied
    // only as a degradation: a strong return is not modelled as better than the datasheet's +-20 mm
    // ranging error, which is dominated by systematic offset rather than by photon count.
    float sigma = std::max(cfg.tof_noise_std_m, best_dist * cfg.tof_noise_pct_of_range);
    float weak = std::sqrt(cfg.tof_signal_threshold_kcps / best_signal);
    sigma *= std::clamp(weak, 1.0f, cfg.tof_weak_signal_sigma_cap);

    if (sigma > 0.0f) {
        std::normal_distribution<float> noise(0.0f, sigma);
        best_dist += noise(rng);
    }
    return best_dist;
}

inline float SampleToFSensorAtMount(Vec2 robot_pos, float robot_theta, const SensorMount& mount,
                                    const std::vector<Wall>& walls, const SimConfig& cfg,
                                    std::mt19937& rng) {
    std::uniform_real_distribution<float> dist(0.0f, 1.0f);
    if (dist(rng) < cfg.tof_dropout_rate) return cfg.tof_max_range_m;

    float world_x = robot_pos.x + std::cos(robot_theta) * mount.x_m - std::sin(robot_theta) * mount.y_m;
    float world_y = robot_pos.y + std::sin(robot_theta) * mount.x_m + std::cos(robot_theta) * mount.y_m;
    float world_angle = robot_theta + (mount.angle_deg * PI_F / 180.0f);
    Vec2 origin{world_x, world_y};

    float reading;
    if (cfg.tof_signal_model) {
        reading = SampleToFSignalModel(origin, world_angle, walls, cfg, rng);
    } else {
        float closest_surface = SampleToFMinOverCone(origin, world_angle, walls, cfg);
        reading = closest_surface;
        float sigma = std::max(cfg.tof_noise_std_m, closest_surface * cfg.tof_noise_pct_of_range);
        if (sigma > 0.0f) {
            std::normal_distribution<float> noise(0.0f, sigma);
            reading += noise(rng);
        }
    }

    // The datasheet minimum ranging distance is 4 cm; below it the sensor still detects a target
    // but does not measure it accurately. The 3 cm floor here is the pre-existing clamp, left as is.
    return std::clamp(reading, cfg.tof_min_range_m, cfg.tof_max_range_m);
}
