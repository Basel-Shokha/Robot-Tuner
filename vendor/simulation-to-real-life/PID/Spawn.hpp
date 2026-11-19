#pragma once
// Spawn.hpp - random start pose for narrow (20-25 cm) corridors. Identical in NN/, PID/, LineFit/.
//
// The robot starts NOT parallel to the walls but still facing down the track, at a random lateral
// position, and never overlapping a wall. The largest heading offset is the one the corridor allows:
// with the axle fixed, turning back toward parallel must never touch a wall, so every heading between 0
// and the offset must fit. The lateral width the body needs at heading psi is
//     extent(psi) = (front + rear) * sin(psi) + width * cos(psi)     (180 x 140 mm body: 180 sin + 140 cos)
// which grows with psi up to ~52 deg, so  phi_max(W) = largest phi with extent(phi) <= W - 2 * spawn_margin_m.
// For the real body and a 10 mm margin: W = 0.20 m -> ~14 deg, W = 0.25 m -> ~31 deg. Turning about the axle
// toward parallel moves both body edges inward, so a pose that is clear stays clear while it straightens.
//
// The local corridor frame comes from the track guide at the track file's spawn point (generators place
// it on a straight lead-in). The pose is checked against THIS run's jittered walls; draws that touch a wall
// are rejected (up to 40 tries, then the file spawn is used).
#include "Robot.hpp"
#include "Track.hpp"
#include <random>

// Largest heading offset (rad) the corridor width W allows, capped at cap_rad.
inline float MaxSpawnHeading(const SimConfig& cfg, float W, float cap_rad) {
    float F = Robot::ChassisFront(cfg), R = Robot::ChassisRear(cfg), Wb = cfg.chassis_width_m;
    float room = W - 2.0f * cfg.spawn_margin_m;
    if (room < Wb) return 0.0f;
    float best = 0.0f;
    for (float a = 0.0f; a <= cap_rad + 1e-6f; a += 0.002f) {  // extent rises monotonically below ~52 deg
        if ((F + R) * std::sin(a) + Wb * std::cos(a) > room) break;
        best = a;
    }
    return best;
}

inline void RandomSpawn(const LoadedTrack& t, const SimConfig& cfg, const std::vector<Wall>& walls, std::mt19937& rng,
                        Vec2& pos, float& heading) {
    pos = Robot::SpawnAxle(cfg, t.spawn, t.spawn_heading);
    heading = t.spawn_heading;
    if (!t.guide.valid) return;
    StraightGuide::Query q = t.guide.At(t.spawn, t.spawn_heading);
    if (!q.ok) return;
    const float W = 2.0f * q.half_width;
    const float cap = cfg.spawn_heading_random_deg * (PI_F / 180.0f);
    const float phi_max = MaxSpawnHeading(cfg, W, cap);
    const float F = Robot::ChassisFront(cfg), R = Robot::ChassisRear(cfg), H = 0.5f * cfg.chassis_width_m;
    const float m = cfg.spawn_margin_m;
    Vec2 u{std::cos(q.tangent), std::sin(q.tangent)}, n{-u.y, u.x};
    Vec2 centre = t.spawn - n * q.center_err;  // nominal corridor centre at the spawn
    std::uniform_real_distribution<float> U(0.0f, 1.0f);
    for (int tries = 0; tries < 40; ++tries) {
        float mag = phi_max * (cfg.spawn_heading_min_frac + (1.0f - cfg.spawn_heading_min_frac) * U(rng));
        float phi = U(rng) < 0.5f ? -mag : mag;
        float sa = std::abs(std::sin(phi)), ca = std::cos(phi);
        // Lateral extent of the body around the axle at this heading (+ = left).
        float hi = (phi > 0 ? F : R) * sa + H * ca, lo = -((phi > 0 ? R : F) * sa + H * ca);
        float ymin = -q.half_width + m - lo, ymax = q.half_width - m - hi;
        if (ymax < ymin) continue;
        float mid = 0.5f * (ymin + ymax), half = 0.5f * (ymax - ymin) * cfg.spawn_lateral_random_frac;
        float y = mid + half * (2.0f * U(rng) - 1.0f);
        Robot probe;
        probe.pos = centre + n * y;
        probe.theta = q.tangent + phi;
        if (probe.ChassisHitsWall(walls, cfg)) continue;
        pos = probe.pos; heading = probe.theta;
        return;
    }
}
