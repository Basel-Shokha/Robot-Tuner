// rt_engine.hpp - the Robot Tuner simulation engine.
//
// A port of the upstream simulator (vendor/simulation-to-real-life/PID, see SOURCE_COMMIT):
//   Robot::Step            -> Sim::Step        (ToF model, glitch filter, reading age, encoders, emergency,
//                                                steering smoother, wheel model "simple" and "pid", slip, collisions)
//   PidBrain::Control      -> Pid::Control     (err = left - right, clip, anti-windup I, filtered D, slowdown)
//   DriveSim (MotorModel)  -> Drive            (firmware wheel PID on the DC-motor model, quantised encoders)
//   PerturbTrack, RandomSpawn, StraightGuide (polyline), EvaluateRobot (train.cpp reward)
// The firmware files wheel_pid.h, emergency.h and steer_smooth.h are #included UNCHANGED from the vendor copy.
//
// Generalised for Robot Tuner:
//   * 1..5 distance sensors, each with its own model (ToF cone min / IR beam / ultrasonic first echo), range,
//     cone, noise, dropout and bias. Left / right sensor for the PID = most CCW / most CW (upstream rule).
//   * 2 wheels, or 4-wheel skid steer (two pairs driven together): same kinematics with the effective track
//     width; four wheel footprints for collisions.
//   * PID "one side" mode: follow one wall at a set distance (robots with one side sensor).
//   * External controller mode (maze algorithms in JS): wheel targets come from outside each tick.
//   * Wall culling (only walls that can be reached are tested; results are unchanged, see Cull()).
//
// With 3 sensors configured like upstream, Sim::Step is bit-identical to upstream Robot::Step
// (tests/native/equiv_upstream.cpp). Not ported (off by default upstream, benchmark experiments only):
// fixed_clock, reflex, slow, speed_law, exp_top_speed / exp_wheel_max.
#pragma once
#include "rt_rng.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <math.h>
#include <string>
#include <vector>

#include "../vendor/simulation-to-real-life/firmware/robot_fw/wheel_pid.h"
#include "../vendor/simulation-to-real-life/firmware/robot_fw/emergency.h"
#if RT_PORTABLE_MATH
#define expf(x) ((float)std::exp((double)(x)))  // steer_smooth.h's only transcendental call, made portable
#endif
#include "../vendor/simulation-to-real-life/firmware/robot_fw/steer_smooth.h"
#if RT_PORTABLE_MATH
#undef expf
#endif

namespace rt {

constexpr float PI = 3.14159265359f;
constexpr int MAXS = 5;          // distance sensors
constexpr int MAX_RAYS = 33;

struct V2 {
    float x{0.0f}, y{0.0f};
    V2 operator+(const V2& o) const { return {x + o.x, y + o.y}; }
    V2 operator-(const V2& o) const { return {x - o.x, y - o.y}; }
    V2 operator*(float s) const { return {x * s, y * s}; }
};
struct Wall { V2 p1, p2; };

enum SensorModel { SM_TOF = 0, SM_IR = 1, SM_ULTRASONIC = 2 };

struct Sensor {
    float x_m = 0.0f, y_m = 0.0f, angle_deg = 0.0f;  // pose in the robot frame (origin = axle midpoint, +y left)
    int model = SM_TOF;
    float half_angle_rad = 0.16816f;  // horizontal half cone
    int rays = 3;
    float max_range_m = 4.0f, min_range_m = 0.03f;
    float noise_std_m = 0.008f, noise_pct = 0.01f;  // sigma = max(std, pct * distance)
    float dropout = 0.02f;
    float bias_m = 0.0f;               // constant offset added to every reading (uncorrected sensor bias)
    float max_incidence_deg = 0.0f;    // ultrasonic: rays hitting a wall more obliquely than this return no echo (0 = off)
};

struct Config {
    float dt = 0.05f;                  // control period (s): the loop period, set by the sensor rate
    // geometry (m)
    float track_width_m = 0.120f;      // effective track width: the kinematics (CAL SPIN)
    float ctrl_track_width_m = 0.0f;   // what the controller believes (<= 0: = track_width_m)
    float wheel_track_geom_m = 0.0f;   // wheel centre to centre for drawing/collision (<= 0: = track_width_m)
    float wheel_radius_m = 0.032f;
    float max_linear_speed_mps = 0.45f;
    float chassis_length_m = 0.10f, chassis_width_m = 0.08f;
    float chassis_front_m = -1.0f, chassis_rear_m = -1.0f;
    float wheel_offset_from_center_m = 0.03f, wheel_length_m = 0.04f, wheel_width_m = 0.015f;
    int four_wheel = 0;                // 1 = skid steer, two wheels per side
    float wheel_base_m = 0.0f;         // 4wd: front-to-rear axle distance
    int n_sensors = 3;
    Sensor sensors[MAXS];
    // ToF / timing
    float tof_glitch_threshold_m = 0.3f;
    int tof_glitch_max_hold_steps = 3;
    float tof_glitch_max_hold_s = 0.0f;
    float tof_latency_s = 0.0f, tof_latency_jitter_s = 0.0f;
    int tof_delay_steps = 0;
    // motors / encoders
    float motor_tau_s = 0.0f, motor_max_accel_mps2 = 0.0f;
    int encoder_model = 0;
    float wheel_slip_coefficient = 0.02f;
    float encoder_noise_base = 0.005f, encoder_noise_speed_coeff = 0.02f;
    int wheel_model = 0;               // 0 simple (lag), 1 firmware wheel PID + DC motor
    float speed_scale = 1.0f;
    int sim_substeps = 10;
    float battery_v = 7.4f, motor_mismatch_std = 0.0f;
    float enc_ticks_per_rev = 1000.0f;
    float pwm_limit = 120.0f, pwm_max = 255.0f, wheel_pid_period_ms = 50.0f;
    float wheel_kp = 0.4f, wheel_ki = 0.001f, wheel_kd = 0.05f, wheel_ff_static = 35.0f, wheel_ff_gain = 2.0f;
    float motor_k_l = 0.00318f, motor_k_r = 0.00318f, motor_static_l = 35.0f, motor_static_r = 35.0f;
    float motor_tau_l_s = 0.08f, motor_tau_r_s = 0.08f, motor_nominal_v = 7.4f;
    float motor_breakaway_l = 0.0f, motor_breakaway_r = 0.0f;
    // per-run hardware bias
    float sensor_angle_bias_std_deg = 0.0f, wheel_scale_bias_std = 0.0f, max_speed_bias_std = 0.0f;
    float wheel_scale_l = 1.0f, wheel_scale_r = 1.0f;  // fixed wheel mismatch (student slider), on top of the bias
    // emergency manoeuvre + smoothing (firmware headers)
    int emerg_on = 1;
    float emerg_front_m = 0.12f, emerg_side_m = 0.075f, emerg_stop_s = 0.10f, emerg_back_speed = 0.08f;
    float emerg_turn_frac = 0.6f, emerg_back_m = 0.06f, emerg_turn_max_rad = 0.35f, emerg_max_s = 1.5f;
    int steer_smooth = 1;
    float steer_tau_s = 0.10f, steer_turn_accel = 0.6f, steer_lin_accel = 0.5f, steer_max_step = 0.01f;
    // PID
    float kp = 1.0f, ki = 0.0f, kd = 0.1f, base_speed_frac = 0.65f;
    float pid_error_clip_m = 0.30f, pid_integral_limit = 0.50f, pid_d_filter_alpha = 0.30f, pid_slowdown_frac = 0.50f;
    int pid_mode = 0;                  // 0 two sides (left - right), 1 follow one wall
    int follow_side = 1;               // one side mode: 1 left wall, -1 right wall
    float follow_dist_m = 0.10f;
    // episode / reward (train.cpp)
    float wall_noise_std_m = 0.010f;
    float max_episode_s = 0.0f; int max_episode_steps = 800;
    float spawn_margin_m = 0.010f, spawn_heading_min_frac = 0.33f;
    float spawn_heading_random_deg = 0.0f, spawn_lateral_random_frac = 0.0f;
    float parallel_weight = 4.0f, center_weight = 6.0f, heading_sigma_deg = 12.0f, center_sigma_frac = 0.30f;
    float crash_penalty = 200.0f, wobble_weight = 60.0f, wobble_omega_deadband = 0.4f;
    float smooth_weight = 5.0f, approach_deadband_mps = 0.25f, finish_bonus = 500.0f, speed_bonus = 0.0f;

    float CtrlTrack() const { return ctrl_track_width_m > 0.0f ? ctrl_track_width_m : track_width_m; }
    float GeomTrack() const { return wheel_track_geom_m > 0.0f ? wheel_track_geom_m : track_width_m; }
    float Front() const { return chassis_front_m > 0.0f ? chassis_front_m : 0.6f * chassis_length_m; }
    float Rear() const { return chassis_rear_m > 0.0f ? chassis_rear_m : 0.4f * chassis_length_m; }
    int MaxSteps() const { return max_episode_s > 0.0f ? (int)std::lround(max_episode_s / dt) : max_episode_steps; }
    int GlitchHoldSteps() const {
        return tof_glitch_max_hold_s > 0.0f ? (int)std::lround(tof_glitch_max_hold_s / dt) : tof_glitch_max_hold_steps;
    }
};

// ---------------------------------------------------------------------------------------- geometry
struct RayHit { float dist = 0.0f, cos_incidence = 0.0f; bool hit = false; };

inline RayHit CastRay(V2 o, float angle, const Wall* const* walls, int nw, float max_range) {
    float rdx = Cos(angle), rdy = Sin(angle);
    RayHit best;
    best.dist = max_range;
    for (int k = 0; k < nw; ++k) {
        const Wall& w = *walls[k];
        float v1x = o.x - w.p1.x, v1y = o.y - w.p1.y;
        float v2x = w.p2.x - w.p1.x, v2y = w.p2.y - w.p1.y;
        float cross = v2x * (-rdy) - v2y * (-rdx);
        if (std::abs(cross) < 1e-6f) continue;
        float t1 = (v2x * v1y - v2y * v1x) / cross;
        float t2 = (v1x * (-rdy) - v1y * (-rdx)) / cross;
        if (t1 >= 0.0f && t2 >= 0.0f && t2 <= 1.0f && t1 < best.dist) {
            best.dist = t1;
            best.hit = true;
            float len = std::sqrt(v2x * v2x + v2y * v2y);
            if (len > 1e-9f) best.cos_incidence = std::abs(rdx * (-v2y / len) + rdy * (v2x / len));
            else best.cos_incidence = 1.0f;
        }
    }
    return best;
}

inline bool SegmentsIntersect(V2 a1, V2 a2, V2 b1, V2 b2) {
    float rx = a2.x - a1.x, ry = a2.y - a1.y;
    float sx = b2.x - b1.x, sy = b2.y - b1.y;
    float cross = rx * sy - ry * sx;
    if (std::abs(cross) < 1e-9f) return false;
    float qx = b1.x - a1.x, qy = b1.y - a1.y;
    float t = (qx * sy - qy * sx) / cross;
    float u = (qx * ry - qy * rx) / cross;
    return t >= 0.0f && t <= 1.0f && u >= 0.0f && u <= 1.0f;
}

// ------------------------------------------------------------------------------------ sensor model
inline int ConeOffsets(int rays, float half, float* out) {
    int n = std::clamp(rays, 1, MAX_RAYS);
    if (n == 1) { out[0] = 0.0f; return 1; }
    for (int i = 0; i < n; ++i) { float t = (float)i / (float)(n - 1); out[i] = (t * 2.0f - 1.0f) * half; }
    return n;
}

// upstream SampleToFSensorAtMount (min over the cone + noise + dropout + clamp), per-sensor parameters.
inline float SampleSensor(V2 rp, float rth, const Sensor& s, float angle_bias_deg, const Wall* const* walls, int nw, Rng& rng) {
    if (Uniform(rng, 0.0f, 1.0f) < s.dropout) return s.max_range_m;
    float wx = rp.x + Cos(rth) * s.x_m - Sin(rth) * s.y_m;
    float wy = rp.y + Sin(rth) * s.x_m + Cos(rth) * s.y_m;
    float wa = rth + ((s.angle_deg + angle_bias_deg) * PI / 180.0f);
    V2 o{wx, wy};
    float off[MAX_RAYS];
    int n = ConeOffsets(s.rays, s.half_angle_rad, off);
    float closest = s.max_range_m;
    const float min_cos = s.max_incidence_deg > 0.0f ? Cos(s.max_incidence_deg * PI / 180.0f) : -1.0f;
    for (int i = 0; i < n; ++i) {
        RayHit h = CastRay(o, wa + off[i], walls, nw, s.max_range_m);
        if (s.model == SM_ULTRASONIC && h.hit && h.cos_incidence < min_cos) continue;  // specular: no echo back
        if (h.dist < closest) closest = h.dist;
    }
    float reading = closest;
    float sigma = std::max(s.noise_std_m, closest * s.noise_pct);
    if (sigma > 0.0f) reading += NormalOnce(rng, 0.0f, sigma);
    if (s.bias_m != 0.0f && closest < s.max_range_m) reading += s.bias_m;
    return std::clamp(reading, s.min_range_m, s.max_range_m);
}

// ---------------------------------------------------------------------------------- PID (PidBrain)
struct Pid {
    float kp = 1.0f, ki = 0.0f, kd = 0.1f, base_speed_frac = 0.65f;
    float error_clip_m = 0.30f, integral_limit = 0.50f, d_filter_alpha = 0.30f, slowdown_frac = 0.50f;
    float max_v = 0.45f;
    int mode = 0, follow_side = 1;
    float follow_dist = 0.1f;
    int left_idx = 2, right_idx = 0;
    float integral = 0.0f, prev_err = 0.0f, d_filtered = 0.0f;
    bool has_prev = false, sat = false;
    // last tick, for plots
    float err = 0.0f, p_term = 0.0f, i_term = 0.0f, d_term = 0.0f, turn = 0.0f, base = 0.0f;

    void Configure(const Config& c) {
        kp = c.kp; ki = c.ki; kd = c.kd; base_speed_frac = c.base_speed_frac;
        error_clip_m = c.pid_error_clip_m; integral_limit = c.pid_integral_limit;
        d_filter_alpha = std::clamp(c.pid_d_filter_alpha, 0.0f, 1.0f);
        slowdown_frac = std::clamp(c.pid_slowdown_frac, 0.0f, 1.0f);
        max_v = c.max_linear_speed_mps;
        mode = c.pid_mode; follow_side = c.follow_side; follow_dist = c.follow_dist_m;
        left_idx = right_idx = 0;
        for (int i = 1; i < c.n_sensors; ++i) {
            if (c.sensors[i].angle_deg > c.sensors[left_idx].angle_deg) left_idx = i;
            if (c.sensors[i].angle_deg < c.sensors[right_idx].angle_deg) right_idx = i;
        }
    }
    void Reset() { integral = 0.0f; prev_err = 0.0f; d_filtered = 0.0f; has_prev = false; }

    void Control(const float* d, float dt, float& v_l, float& v_r) {
        float raw;
        if (mode == 1) raw = follow_side > 0 ? d[left_idx] - follow_dist : follow_dist - d[right_idx];
        else raw = d[left_idx] - d[right_idx];
        float e = std::clamp(raw, -error_clip_m, error_clip_m);
        integral = std::clamp(integral + e * dt, -integral_limit, integral_limit);
        float raw_d = has_prev ? (e - prev_err) / dt : 0.0f;
        float alpha = 1.0f - Pow(1.0f - d_filter_alpha, dt / 0.05f);
        d_filtered = alpha * raw_d + (1.0f - alpha) * d_filtered;
        prev_err = e;
        has_prev = true;
        float t = kp * e + ki * integral + kd * d_filtered;
        t = std::clamp(t, -max_v, max_v);
        float b = base_speed_frac * max_v * (1.0f - slowdown_frac * std::abs(t) / max_v);
        v_l = std::clamp(b - t, -max_v, max_v);
        v_r = std::clamp(b + t, -max_v, max_v);
        sat = std::abs((kp * e + ki * integral + kd * d_filtered) - 0.5f * (v_r - v_l)) > 0.002f;
        err = e; p_term = kp * e; i_term = ki * integral; d_term = kd * d_filtered; turn = t; base = b;
    }
};

// ------------------------------------------------------------------------ inner loop (DriveSim)
struct Drive {
    float w[2] = {0.0f, 0.0f};
    double dist[2] = {0.0, 0.0};
    long long tick_prev_pid[2] = {0, 0}, tick_prev_ctrl[2] = {0, 0};
    fw::WheelPid pid[2];
    int pwm[2] = {0, 0};
    float pid_timer = 0.0f;

    void Reset() { *this = Drive(); }
    static float MetersPerTick(const Config& c) { return fw::MetersPerTick(2.0f * c.wheel_radius_m, c.enc_ticks_per_rev, 1.0f); }
    long long Ticks(int i, const Config& c) const { return (long long)std::floor(dist[i] / MetersPerTick(c)); }
    void ReadControlEncoders(const Config& c, float dt, float& el, float& er) {
        long long t0 = Ticks(0, c), t1 = Ticks(1, c);
        float mpt = MetersPerTick(c);
        el = (float)(t0 - tick_prev_ctrl[0]) * mpt / dt;
        er = (float)(t1 - tick_prev_ctrl[1]) * mpt / dt;
        tick_prev_ctrl[0] = t0; tick_prev_ctrl[1] = t1;
    }
    static fw::WheelPidGains Gains(const Config& c) {
        fw::WheelPidGains g;
        g.kp = c.wheel_kp; g.ki = c.wheel_ki; g.kd = c.wheel_kd;
        g.ff_static = c.wheel_ff_static; g.ff_gain = c.wheel_ff_gain;
        g.pwm_limit = c.pwm_limit; g.pwm_max = c.pwm_max;
        return g;
    }
    static float SteadySpeed(const Config& c, int i, int p, float gain, bool at_rest) {
        float k = i == 0 ? c.motor_k_l : c.motor_k_r;
        float st = i == 0 ? c.motor_static_l : c.motor_static_r;
        float brk = i == 0 ? c.motor_breakaway_l : c.motor_breakaway_r;
        float a = std::abs((float)p);
        if (a <= st || (at_rest && a <= brk)) return 0.0f;
        return (p > 0 ? 1.0f : -1.0f) * k * (a - st) * gain;
    }
    template <class Pose>
    void Step(const Config& c, float dt, const float tg_mps[2], const float gain[2], const float scale[2], Pose& pose, float ground[2]) {
        int n = std::max(1, c.sim_substeps);
        float h = dt / n;
        float mpt = MetersPerTick(c);
        float period = std::max(0.001f, c.wheel_pid_period_ms / 1000.0f);
        float tgt[2] = {fw::MpsToTp(tg_mps[0], mpt, period), fw::MpsToTp(tg_mps[1], mpt, period)};
        float tau[2] = {std::max(1e-4f, c.motor_tau_l_s), std::max(1e-4f, c.motor_tau_r_s)};
        fw::WheelPidGains g = Gains(c);
        ground[0] = ground[1] = 0.0f;
        for (int s = 0; s < n; ++s) {
            if (pid_timer <= 1e-6f) {
                for (int i = 0; i < 2; ++i) {
                    long long t = Ticks(i, c);
                    float meas = (float)(t - tick_prev_pid[i]);
                    tick_prev_pid[i] = t;
                    pwm[i] = pid[i].Step(g, true, tgt[i], meas);
                }
                pid_timer += period;
            }
            pid_timer -= h;
            float vg[2];
            for (int i = 0; i < 2; ++i) {
                float prev = w[i];
                float ss = SteadySpeed(c, i, pwm[i], gain[i], std::abs(w[i]) < 1e-4f);
                w[i] += (ss - w[i]) * (1.0f - Exp(-h / tau[i]));
                dist[i] += (double)w[i] * h;
                float slip = 1.0f - std::min(0.15f, c.wheel_slip_coefficient * std::abs(w[i] - prev) / h);
                vg[i] = w[i] * slip * scale[i];
                ground[i] += vg[i] / n;
            }
            pose.Integrate(vg[0], vg[1], c.track_width_m, h);
        }
    }
};

// -------------------------------------------------------------------- guide (StraightGuide polyline)
struct Guide {
    bool valid = false;
    std::vector<V2> pts;
    std::vector<float> hw, cs;
    struct Query { bool ok = false; float s = 0, center_err = 0, half_width = 0, heading_err = 0, tangent = 0; };
    void Build(const std::vector<V2>& p, const std::vector<float>& h) {
        valid = false;
        if (p.size() < 2) return;
        pts = p; hw = h; cs.assign(p.size(), 0.0f);
        for (size_t i = 1; i < p.size(); ++i) cs[i] = cs[i - 1] + Hypot(p[i].x - p[i - 1].x, p[i].y - p[i - 1].y);
        valid = true;
    }
    static float WrapPi(float a) { while (a > PI) a -= 2.0f * PI; while (a < -PI) a += 2.0f * PI; return a; }
    // hint: segment index to start a local search from (-1 = full search). Returns the segment used.
    Query At(V2 pos, float theta, int* hint = nullptr) const {
        Query q;
        if (!valid) return q;
        const size_t last = pts.size() - 1;
        size_t lo = 0, hi = last;
        if (hint && *hint >= 0) { lo = (size_t)std::max(0, *hint - 40); hi = std::min(last, (size_t)*hint + 41); }
        float best = 1e30f;
        int bi = Search(pos, lo, hi, q, best);
        // the nearest segment sits at the edge of the local window (e.g. the seam of a closed loop): search all
        if (hint && *hint >= 0 && (lo > 0 || hi < last) && (bi == (int)lo || bi == (int)hi - 1)) {
            Query full; float fb = 1e30f;
            int fi = Search(pos, 0, last, full, fb);
            if (fb < best) { q = full; best = fb; bi = fi; }
        }
        if (hint) *hint = bi;
        q.heading_err = WrapPi(theta - q.tangent);
        q.ok = best < 1e29f;
        return q;
    }
    int Search(V2 pos, size_t lo, size_t hi, Query& q, float& best) const {
        int bi = -1;
        for (size_t i = lo; i < hi; ++i) {
            V2 a = pts[i], d = pts[i + 1] - pts[i];
            float L2 = d.x * d.x + d.y * d.y;
            if (L2 < 1e-12f) continue;
            float t = std::clamp(((pos.x - a.x) * d.x + (pos.y - a.y) * d.y) / L2, 0.0f, 1.0f);
            float px = a.x + d.x * t - pos.x, py = a.y + d.y * t - pos.y;
            float dist2 = px * px + py * py;
            if (dist2 < best) {
                best = dist2; bi = (int)i;
                float L = std::sqrt(L2);
                q.s = cs[i] + t * L;
                q.tangent = Atan2(d.y, d.x);
                q.center_err = ((pos.x - a.x) * (-d.y) + (pos.y - a.y) * d.x) / L;
                q.half_width = hw[i] + (hw[i + 1] - hw[i]) * t;
            }
        }
        return bi;
    }
};

// ------------------------------------------------------------- PerturbTrack (upstream, verbatim logic)
inline std::vector<Wall> PerturbTrack(const std::vector<Wall>& track, float noise_std, Rng& rng) {
    if (noise_std <= 0.0f) return track;
    const float KNOT_M = 0.30f, SAME = 1e-4f;
    Normal dist(0.0f, noise_std);
    std::vector<V2> V;
    std::vector<int> wv(2 * track.size());
    auto vid = [&](V2 p) {
        for (size_t i = 0; i < V.size(); ++i)
            if (std::abs(V[i].x - p.x) < SAME && std::abs(V[i].y - p.y) < SAME) return (int)i;
        V.push_back(p);
        return (int)V.size() - 1;
    };
    for (size_t i = 0; i < track.size(); ++i) { wv[2 * i] = vid(track[i].p1); wv[2 * i + 1] = vid(track[i].p2); }
    std::vector<std::vector<int>> adj(V.size());
    for (size_t i = 0; i < track.size(); ++i) { adj[wv[2 * i]].push_back((int)i); adj[wv[2 * i + 1]].push_back((int)i); }
    std::vector<V2> off(V.size());
    std::vector<char> vdone(V.size(), 0), wdone(track.size(), 0);
    auto walk = [&](int start) {
        V2 k0{dist(rng), dist(rng)}, k1{dist(rng), dist(rng)};
        float s = 0.0f, knot_s = 0.0f;
        if (!vdone[start]) { off[start] = k0; vdone[start] = 1; } else { k0 = off[start]; }
        int v = start;
        for (;;) {
            int w = -1;
            for (int c : adj[v]) if (!wdone[c]) { w = c; break; }
            if (w < 0) return;
            wdone[w] = 1;
            int u = wv[2 * w] == v ? wv[2 * w + 1] : wv[2 * w];
            s += Hypot(V[u].x - V[v].x, V[u].y - V[v].y);
            while (s > knot_s + KNOT_M) { knot_s += KNOT_M; k0 = k1; k1 = {dist(rng), dist(rng)}; }
            float f = std::clamp((s - knot_s) / KNOT_M, 0.0f, 1.0f);
            if (!vdone[u]) { off[u] = {k0.x + (k1.x - k0.x) * f, k0.y + (k1.y - k0.y) * f}; vdone[u] = 1; }
            v = u;
        }
    };
    for (size_t i = 0; i < V.size(); ++i) if (adj[i].size() == 1 && !vdone[i]) walk((int)i);
    for (size_t i = 0; i < V.size(); ++i) if (!vdone[i]) walk((int)i);
    std::vector<Wall> out = track;
    for (size_t i = 0; i < track.size(); ++i) { out[i].p1 = track[i].p1 + off[wv[2 * i]]; out[i].p2 = track[i].p2 + off[wv[2 * i + 1]]; }
    return out;
}

// ----------------------------------------------------------------------------------- the robot
enum State { NAV = 0, EMERG_STOP = 1, EMERG_REVERSE = 2 };

struct Sim {
    Config cfg;
    std::vector<Wall> walls;
    std::vector<const Wall*> near;  // culled wall list for this tick
    bool cull = true;
    // pose / commands
    V2 pos; float theta = 0.0f;
    float v_l = 0.0f, v_r = 0.0f;
    float dists[MAXS], raw[MAXS], last_valid[MAXS];
    int glitch_hold[MAXS];
    static constexpr int TOF_HIST = 4, POSE_HIST = 8;
    float tof_hist[TOF_HIST][MAXS];
    int tof_hist_n = 0;
    V2 pose_hist_p[POSE_HIST]; float pose_hist_th[POSE_HIST]; int pose_hist_n = 0;
    float wheel_l = 0, wheel_r = 0, enc_l = 0, enc_r = 0, last_eff_l = 0, last_eff_r = 0;
    double enc_dist_l = 0, enc_dist_r = 0;  // what the encoders counted, m (external controllers)
    bool alive = true, finished = false, scan_motion = false;
    float scan_rotation_accum = 0.0f;
    int state = NAV;
    long steps = 0;
    emerg::Logic emergency;
    smooth::Filter steer;
    Pid pid;
    Drive drive;
    float pack_scale = 1.0f, motor_mismatch[2] = {1.0f, 1.0f};
    float sensor_angle_bias_deg[MAXS];
    float wheel_scale_l = 1.0f, wheel_scale_r = 1.0f;
    // external controller (maze): targets for the next Step, and whether to smooth them
    bool external = false; float ext_l = 0, ext_r = 0;
    int idx_r = 0, idx_f = 1, idx_l = 2;  // sensors used by the emergency logic (right, front, left)

    Sim() { ResetArrays(4.0f); }
    void ResetArrays(float r) {
        for (int i = 0; i < MAXS; ++i) {
            dists[i] = raw[i] = last_valid[i] = r; glitch_hold[i] = 0; sensor_angle_bias_deg[i] = 0.0f;
            for (int k = 0; k < TOF_HIST; ++k) tof_hist[k][i] = 0.0f;
        }
    }

    float MaxRange(int i) const { return cfg.sensors[i].max_range_m; }

    void Reset(V2 p, float heading) {
        pos = p; theta = heading; v_l = v_r = 0.0f;
        alive = true; finished = false; state = NAV; steps = 0;
        emergency.Reset(); steer.Reset();
        for (int i = 0; i < MAXS; ++i) {
            float r = i < cfg.n_sensors ? MaxRange(i) : 4.0f;
            dists[i] = raw[i] = last_valid[i] = r; glitch_hold[i] = 0; sensor_angle_bias_deg[i] = 0.0f;
        }
        scan_rotation_accum = 0.0f;
        wheel_scale_l = cfg.wheel_scale_l; wheel_scale_r = cfg.wheel_scale_r;
        wheel_l = wheel_r = enc_l = enc_r = last_eff_l = last_eff_r = 0.0f;
        enc_dist_l = enc_dist_r = 0.0;
        tof_hist_n = 0; scan_motion = false; pose_hist_n = 0;
        drive.Reset();
        pack_scale = 1.0f; motor_mismatch[0] = motor_mismatch[1] = 1.0f;
        pid.Configure(cfg);
        pid.Reset();
        // emergency sensors: most CW = right, most CCW = left, smallest |angle| = front
        idx_r = pid.right_idx; idx_l = pid.left_idx; idx_f = 0;
        for (int i = 1; i < cfg.n_sensors; ++i) if (std::abs(cfg.sensors[i].angle_deg) < std::abs(cfg.sensors[idx_f].angle_deg)) idx_f = i;
    }

    void SampleHardwareBias(Rng& rng) {
        if (cfg.sensor_angle_bias_std_deg > 0.0f) {
            Normal d(0.0f, cfg.sensor_angle_bias_std_deg);
            for (int i = 0; i < cfg.n_sensors; ++i) sensor_angle_bias_deg[i] = d(rng);
        }
        if (cfg.wheel_scale_bias_std > 0.0f) {
            Normal d(0.0f, cfg.wheel_scale_bias_std);
            wheel_scale_l = std::clamp(1.0f + d(rng), 0.5f, 1.5f) * cfg.wheel_scale_l;
            wheel_scale_r = std::clamp(1.0f + d(rng), 0.5f, 1.5f) * cfg.wheel_scale_r;
        }
        if (cfg.max_speed_bias_std > 0.0f) {
            Normal d(0.0f, cfg.max_speed_bias_std);
            float pack = std::clamp(1.0f + d(rng), 0.5f, 1.2f);
            if (cfg.wheel_model == 1) pack_scale = pack;
            else { wheel_scale_l *= pack; wheel_scale_r *= pack; }
        }
        if (cfg.wheel_model == 1 && cfg.motor_mismatch_std > 0.0f) {
            Normal d(0.0f, cfg.motor_mismatch_std);
            motor_mismatch[0] = std::clamp(1.0f + d(rng), 0.5f, 1.5f);
            motor_mismatch[1] = std::clamp(1.0f + d(rng), 0.5f, 1.5f);
        }
    }

    // --- footprint
    void BodyCorners(V2 c[4]) const {
        float rear = -cfg.Rear(), front = cfg.Front(), hw = 0.5f * cfg.chassis_width_m;
        const float L[4][2] = {{front, -hw}, {front, hw}, {rear, hw}, {rear, -hw}};
        float co = Cos(theta), si = Sin(theta);
        for (int i = 0; i < 4; ++i) c[i] = {pos.x + co * L[i][0] - si * L[i][1], pos.y + si * L[i][0] + co * L[i][1]};
    }
    int NumWheels() const { return cfg.four_wheel ? 4 : 2; }
    void WheelCorners(int k, V2 c[4]) const {
        int side = k % 2;  // 0 left, 1 right
        float cx = -cfg.wheel_offset_from_center_m;
        if (cfg.four_wheel) cx += (k < 2 ? 0.5f : -0.5f) * cfg.wheel_base_m;
        float cy = (side == 0 ? 0.5f : -0.5f) * cfg.GeomTrack();
        float hl = 0.5f * cfg.wheel_length_m, hw = 0.5f * cfg.wheel_width_m;
        const float L[4][2] = {{cx + hl, cy - hw}, {cx + hl, cy + hw}, {cx - hl, cy + hw}, {cx - hl, cy - hw}};
        float co = Cos(theta), si = Sin(theta);
        for (int i = 0; i < 4; ++i) c[i] = {pos.x + co * L[i][0] - si * L[i][1], pos.y + si * L[i][0] + co * L[i][1]};
    }
    static bool Touches(const V2 c[4], const Wall& w) {
        for (int i = 0; i < 4; ++i) if (SegmentsIntersect(c[i], c[(i + 1) % 4], w.p1, w.p2)) return true;
        return false;
    }
    bool HitsWall() const {
        V2 parts[5][4];
        BodyCorners(parts[0]);
        int np = 1 + NumWheels();
        for (int k = 0; k < NumWheels(); ++k) WheelCorners(k, parts[1 + k]);
        for (const Wall* w : near)
            for (int p = 0; p < np; ++p) if (Touches(parts[p], *w)) return true;
        return false;
    }
    float FootprintRadius() const {
        float r = std::max(Hypot(cfg.Front(), 0.5f * cfg.chassis_width_m), Hypot(cfg.Rear(), 0.5f * cfg.chassis_width_m));
        float wx = std::abs(cfg.wheel_offset_from_center_m) + 0.5f * cfg.wheel_length_m + (cfg.four_wheel ? 0.5f * cfg.wheel_base_m : 0.0f);
        float wy = 0.5f * cfg.GeomTrack() + 0.5f * cfg.wheel_width_m;
        r = std::max(r, Hypot(wx, wy));
        for (int i = 0; i < cfg.n_sensors; ++i) r = std::max(r, Hypot(cfg.sensors[i].x_m, cfg.sensors[i].y_m));
        return r;
    }

    // Walls that can matter this tick: within reach of any ray (max range from a sensor that is within the
    // footprint radius of the robot, plus how far a delayed reading's pose can lie behind) or the body.
    // Walls farther away cannot produce a ray hit closer than the max range nor touch the body, so dropping
    // them changes no result.
    void Cull() {
        near.clear();
        if (!cull) { for (const Wall& w : walls) near.push_back(&w); return; }
        float mr = 0.0f;
        for (int i = 0; i < cfg.n_sensors; ++i) mr = std::max(mr, MaxRange(i));
        float lag = (cfg.tof_latency_s + cfg.tof_latency_jitter_s + cfg.tof_delay_steps * cfg.dt + 2.0f * cfg.dt) * 2.0f;
        float R = mr + FootprintRadius() + lag + 0.05f;
        for (const Wall& w : walls) {
            float minx = std::min(w.p1.x, w.p2.x), maxx = std::max(w.p1.x, w.p2.x);
            float miny = std::min(w.p1.y, w.p2.y), maxy = std::max(w.p1.y, w.p2.y);
            float dx = pos.x < minx ? minx - pos.x : (pos.x > maxx ? pos.x - maxx : 0.0f);
            float dy = pos.y < miny ? miny - pos.y : (pos.y > maxy ? pos.y - maxy : 0.0f);
            if (dx <= R && dy <= R && dx * dx + dy * dy <= R * R) near.push_back(&w);
        }
    }

    float DelayedToF(int i, float fresh) {
        int delay = std::clamp(cfg.tof_delay_steps, 0, TOF_HIST - 1);
        if (delay == 0) return fresh;
        if (i == 0) {
            for (int k = TOF_HIST - 1; k > 0; --k) for (int j = 0; j < MAXS; ++j) tof_hist[k][j] = tof_hist[k - 1][j];
            tof_hist_n = std::min(tof_hist_n + 1, TOF_HIST);
        }
        tof_hist[0][i] = fresh;
        int k = std::min(delay, tof_hist_n - 1);
        return tof_hist[k][i];
    }
    void PastPose(float age, V2& p, float& th) const {
        if (pose_hist_n <= 1 || age <= 0.0f) { p = pos; th = theta; return; }
        float k = age / cfg.dt;
        int a = std::min((int)k, pose_hist_n - 1);
        int b = std::min(a + 1, pose_hist_n - 1);
        float f = std::clamp(k - (float)a, 0.0f, 1.0f);
        if (a == b) f = 0.0f;
        p = {pose_hist_p[a].x + f * (pose_hist_p[b].x - pose_hist_p[a].x), pose_hist_p[a].y + f * (pose_hist_p[b].y - pose_hist_p[a].y)};
        float dth = pose_hist_th[b] - pose_hist_th[a];
        while (dth > PI) dth -= 2.0f * PI;
        while (dth < -PI) dth += 2.0f * PI;
        th = pose_hist_th[a] + f * dth;
    }
    float MotorResponse(float wheel, float cmd, float dt) const {
        float next = cfg.motor_tau_s > 0.0f ? wheel + (cmd - wheel) * (1.0f - Exp(-dt / cfg.motor_tau_s)) : cmd;
        if (cfg.motor_max_accel_mps2 > 0.0f) { float dv = cfg.motor_max_accel_mps2 * dt; next = std::clamp(next, wheel - dv, wheel + dv); }
        return next;
    }
    struct PoseRef {
        V2& pos; float& theta;
        void Integrate(float vl, float vr, float track, float h) {
            float v = 0.5f * (vl + vr), w = (vr - vl) / track;
            pos.x += v * Cos(theta) * h;
            pos.y += v * Sin(theta) * h;
            theta += w * h;
        }
    };

    void Step(Rng& rng, const Wall* finish = nullptr) {
        if (!alive) return;
        const float DT = cfg.dt;
        for (int k = POSE_HIST - 1; k > 0; --k) { pose_hist_p[k] = pose_hist_p[k - 1]; pose_hist_th[k] = pose_hist_th[k - 1]; }
        pose_hist_p[0] = pos; pose_hist_th[0] = theta;
        pose_hist_n = std::min(pose_hist_n + 1, POSE_HIST);
        Cull();
        const Wall* const* W = near.data();
        const int NW = (int)near.size();

        for (int i = 0; i < cfg.n_sensors; ++i) {
            V2 sp = pos; float sth = theta;
            if (cfg.tof_latency_s > 0.0f) {
                float age = cfg.tof_latency_s;
                if (cfg.tof_latency_jitter_s > 0.0f) age += Uniform(rng, 0.0f, cfg.tof_latency_jitter_s);
                PastPose(age, sp, sth);
            }
            float r = DelayedToF(i, SampleSensor(sp, sth, cfg.sensors[i], sensor_angle_bias_deg[i], W, NW, rng));
            raw[i] = r;
            if (r > last_valid[i] && (r - last_valid[i]) > cfg.tof_glitch_threshold_m && glitch_hold[i] < cfg.GlitchHoldSteps()) {
                dists[i] = last_valid[i];
                ++glitch_hold[i];
            } else {
                dists[i] = r;
                last_valid[i] = r;
                glitch_hold[i] = 0;
            }
        }

        if (HitsWall()) { alive = false; return; }

        const float ctrl_track = cfg.CtrlTrack();
        if (cfg.wheel_model == 1) {
            drive.ReadControlEncoders(cfg, DT, enc_l, enc_r);
            if (scan_motion) scan_rotation_accum += (enc_r - enc_l) / ctrl_track * DT;
        } else {
            float base_l = cfg.encoder_model == 1 ? wheel_l * wheel_scale_l : v_l;
            float base_r = cfg.encoder_model == 1 ? wheel_r * wheel_scale_r : v_r;
            float sl = cfg.encoder_noise_base + cfg.encoder_noise_speed_coeff * std::abs(base_l);
            float sr = cfg.encoder_noise_base + cfg.encoder_noise_speed_coeff * std::abs(base_r);
            enc_l = base_l; enc_r = base_r;
            if (sl > 0.0f) enc_l += NormalOnce(rng, 0.0f, sl);
            if (sr > 0.0f) enc_r += NormalOnce(rng, 0.0f, sr);
            if (cfg.encoder_model == 1 && scan_motion) scan_rotation_accum += (enc_r - enc_l) / ctrl_track * DT;
        }
        enc_dist_l += (double)enc_l * DT; enc_dist_r += (double)enc_r * DT;

        if (external) {
            state = NAV;
            v_l = ext_l; v_r = ext_r;
            steer.p.enabled = cfg.steer_smooth; steer.p.turn_tau_s = cfg.steer_tau_s;
            steer.p.turn_accel = cfg.steer_turn_accel; steer.p.lin_accel = cfg.steer_lin_accel; steer.p.max_step = cfg.steer_max_step;
            steer.Apply(v_l, v_r, DT);
        } else {
            float d3[3] = {Read(idx_r), Read(idx_f), Read(idx_l)};
            emergency.p.enabled = cfg.emerg_on; emergency.p.front_m = cfg.emerg_front_m; emergency.p.side_m = cfg.emerg_side_m;
            emergency.p.stop_s = cfg.emerg_stop_s; emergency.p.back_speed = cfg.emerg_back_speed;
            emergency.p.turn_frac = cfg.emerg_turn_frac; emergency.p.back_m = cfg.emerg_back_m;
            emergency.p.turn_max_rad = cfg.emerg_turn_max_rad; emergency.p.max_s = cfg.emerg_max_s;
            emergency.p.track_w = ctrl_track;
            bool ended = false;
            bool active = emergency.Step(d3, enc_l, enc_r, DT, v_l, v_r, ended);
            if (ended) { steer.Reset(); pid.Reset(); }
            if (active) {
                state = emergency.state == emerg::STOP ? EMERG_STOP : EMERG_REVERSE;
            } else {
                state = NAV;
                pid.Control(dists, DT, v_l, v_r);
                steer.p.enabled = cfg.steer_smooth; steer.p.turn_tau_s = cfg.steer_tau_s;
                steer.p.turn_accel = cfg.steer_turn_accel; steer.p.lin_accel = cfg.steer_lin_accel; steer.p.max_step = cfg.steer_max_step;
                steer.Apply(v_l, v_r, DT);
                emerg::NoReverse(v_l, v_r);
            }
        }

        if (cfg.wheel_model == 1) {
            float targets[2] = {v_l * cfg.speed_scale, v_r * cfg.speed_scale};
            float base_gain = cfg.battery_v / cfg.motor_nominal_v * pack_scale;
            float gain[2] = {base_gain * motor_mismatch[0], base_gain * motor_mismatch[1]};
            float scale[2] = {wheel_scale_l, wheel_scale_r};
            float ground[2];
            PoseRef pr{pos, theta};
            drive.Step(cfg, DT, targets, gain, scale, pr, ground);
            wheel_l = drive.w[0]; wheel_r = drive.w[1];
            last_eff_l = ground[0]; last_eff_r = ground[1];
        } else {
            float pwl = wheel_l, pwr = wheel_r;
            wheel_l = MotorResponse(wheel_l, v_l * cfg.speed_scale, DT);
            wheel_r = MotorResponse(wheel_r, v_r * cfg.speed_scale, DT);
            float al = std::abs(wheel_l - pwl) / DT, ar = std::abs(wheel_r - pwr) / DT;
            float sfl = 1.0f - std::min(0.15f, cfg.wheel_slip_coefficient * al);
            float sfr = 1.0f - std::min(0.15f, cfg.wheel_slip_coefficient * ar);
            float vel = wheel_l * sfl * wheel_scale_l, ver = wheel_r * sfr * wheel_scale_r;
            last_eff_l = vel; last_eff_r = ver;
            float v = (vel + ver) * 0.5f;
            float om = (ver - vel) / cfg.track_width_m;
            pos.x += v * Cos(theta) * DT;
            pos.y += v * Sin(theta) * DT;
            theta += om * DT;
            if (cfg.encoder_model != 1 && state != NAV) scan_rotation_accum += om * DT;
        }
        scan_motion = state != NAV;
        while (theta > PI) theta -= 2.0f * PI;
        while (theta <= -PI) theta += 2.0f * PI;
        ++steps;
        if (finish && !finished) {
            V2 c[4];
            BodyCorners(c);
            if (Touches(c, *finish)) finished = true;
        }
    }
    float Read(int i) const { return i < cfg.n_sensors ? dists[i] : 4.0f; }
};

// -------------------------------------------------------------------------- random spawn (Spawn.hpp)
inline float MaxSpawnHeading(const Config& c, float W, float cap) {
    float F = c.Front(), R = c.Rear(), Wb = c.chassis_width_m;
    float room = W - 2.0f * c.spawn_margin_m;
    if (room < Wb) return 0.0f;
    float best = 0.0f;
    for (float a = 0.0f; a <= cap + 1e-6f; a += 0.002f) {
        if ((F + R) * Sin(a) + Wb * Cos(a) > room) break;
        best = a;
    }
    return best;
}

// Like upstream RandomSpawn; cap_deg is the student's "start rotated up to" slider. The heading limit is the
// smaller of the cap and what the corridor width allows (a pose that touches a wall is never used).
inline void RandomSpawn(Sim& probe, const Guide& g, V2 spawn, float spawn_heading, Rng& rng, V2& pos, float& heading) {
    const Config& c = probe.cfg;
    pos = spawn; heading = spawn_heading;
    if (!g.valid) return;
    Guide::Query q = g.At(spawn, spawn_heading);
    if (!q.ok) return;
    const float W = 2.0f * q.half_width;
    const float cap = c.spawn_heading_random_deg * (PI / 180.0f);
    const float phi_max = MaxSpawnHeading(c, W, cap);
    const float F = c.Front(), R = c.Rear(), H = 0.5f * c.chassis_width_m, m = c.spawn_margin_m;
    V2 u{Cos(q.tangent), Sin(q.tangent)}, n{-u.y, u.x};
    V2 centre = spawn - n * q.center_err;
    for (int tries = 0; tries < 40; ++tries) {
        float mag = phi_max * (c.spawn_heading_min_frac + (1.0f - c.spawn_heading_min_frac) * Uniform(rng, 0.0f, 1.0f));
        float phi = Uniform(rng, 0.0f, 1.0f) < 0.5f ? -mag : mag;
        float sa = std::abs(Sin(phi)), ca = Cos(phi);
        float hi = (phi > 0 ? F : R) * sa + H * ca, lo = -((phi > 0 ? R : F) * sa + H * ca);
        float ymin = -q.half_width + m - lo, ymax = q.half_width - m - hi;
        if (ymax < ymin) continue;
        float mid = 0.5f * (ymin + ymax), half = 0.5f * (ymax - ymin) * c.spawn_lateral_random_frac;
        float y = mid + half * (2.0f * Uniform(rng, 0.0f, 1.0f) - 1.0f);
        probe.pos = centre + n * y;
        probe.theta = q.tangent + phi;
        probe.Cull();
        if (probe.HitsWall()) continue;
        pos = probe.pos; heading = probe.theta;
        return;
    }
}

// ------------------------------------------------------------------------ episode (train.cpp reward)
struct Track {
    std::vector<Wall> walls;
    Guide guide;
    V2 spawn; float spawn_heading = 0.0f;
    bool has_finish = false; Wall finish;
};

struct EpisodeStats {
    float score = 0;
    int crashed = 0, finished = 0, steps = 0;
    float forward_m = 0, mean_abs_center = 0, mean_abs_heading = 0, rms_center = 0, time_s = 0;
};

constexpr int EPISODES_PER_TRACK = 4;

// upstream EvaluateRobot (train.cpp), same seeds and the same reward terms. Mean centring/heading errors are
// extra statistics for the report (they do not enter the score).
inline EpisodeStats Evaluate(const Config& cfg, const Track& t, unsigned run_seed, int generation, int episode) {
    EpisodeStats st;
    Rng rng(SeedMix(run_seed, 42u + (unsigned)episode * 7919u));
    Rng track_rng(SeedMix(run_seed ^ 0xABCDu, (unsigned)(generation * EPISODES_PER_TRACK + episode)));
    Sim bot;
    bot.cfg = cfg;
    bot.walls = PerturbTrack(t.walls, cfg.wall_noise_std_m, track_rng);
    V2 sp = t.spawn; float sh = t.spawn_heading;
    if (episode > 0) { bot.Reset(sp, sh); RandomSpawn(bot, t.guide, t.spawn, t.spawn_heading, track_rng, sp, sh); }
    bot.Reset(sp, sh);
    bot.SampleHardwareBias(rng);

    float progress = 0, forward_m = 0, backward_m = 0, action_pen = 0, approach_pen = 0, wobble_pen = 0;
    int last_turn = 0;
    bool died = false, finished = false;
    int used = 0;
    float pal = 0, par = 0, prev_min = 0;
    const int MAX_STEPS = cfg.MaxSteps();
    const float PW = 100.0f, HALF_W = 0.5f * cfg.chassis_width_m;
    const float hs = cfg.heading_sigma_deg * (PI / 180.0f);
    const float DT = cfg.dt;
    int hint = -1;
    int* hp = t.guide.pts.size() > 200 ? &hint : nullptr;  // long tracks: local search (upstream: always full)
    float prev_s = t.guide.At(bot.pos, bot.theta, hp).s;
    double sum_c = 0, sum_c2 = 0, sum_h = 0; int nq = 0;
    const Wall* fin = t.has_finish ? &t.finish : nullptr;
    for (int step = 0; step < MAX_STEPS; ++step) {
        bot.Step(rng, fin);
        used = step + 1;
        if (!bot.alive) { died = true; break; }
        Guide::Query q = t.guide.At(bot.pos, bot.theta, hp);
        if (q.ok) {
            float ds = q.s - prev_s;
            prev_s = q.s;
            if (ds > 0.0f) {
                forward_m += ds;
                float p = Exp(-(q.heading_err * q.heading_err) / (hs * hs));
                float fh = std::max(0.02f, q.half_width - HALF_W);
                float e = q.center_err / (cfg.center_sigma_frac * fh);
                float c = Exp(-e * e);
                progress += ds * (1.0f + cfg.parallel_weight * p + cfg.center_weight * c);
            } else backward_m -= ds;
            sum_c += std::abs(q.center_err); sum_c2 += (double)q.center_err * q.center_err; sum_h += std::abs(q.heading_err); ++nq;
        }
        float al = bot.v_l / cfg.max_linear_speed_mps, ar = bot.v_r / cfg.max_linear_speed_mps;
        if (step > 0) { float dl = al - pal, dr = ar - par; action_pen += (dl * dl + dr * dr) * (0.05f / DT); }
        pal = al; par = ar;
        float mn = bot.dists[0];
        for (int i = 1; i < cfg.n_sensors; ++i) mn = std::min(mn, bot.dists[i]);
        if (step > 0) {
            float dmin = (mn - prev_min) / DT;
            const float db = cfg.approach_deadband_mps * (0.05f / DT);
            if (dmin < -db) { float cl = std::min(-dmin - db, 1.0f); approach_pen += cl * cl * (DT / 0.05f); }
        }
        prev_min = mn;
        if (bot.state == NAV) {
            float om = (bot.last_eff_r - bot.last_eff_l) / cfg.CtrlTrack();
            if (std::abs(om) > cfg.wobble_omega_deadband) {
                int sg = om > 0.0f ? 1 : -1;
                if (last_turn != 0 && sg != last_turn) wobble_pen += std::min(std::abs(om), 3.0f);
                last_turn = sg;
            }
        } else last_turn = 0;
        if (bot.finished) { finished = true; break; }
    }
    float score = progress * PW - backward_m * PW * (1.0f + cfg.parallel_weight + cfg.center_weight)
                - action_pen * cfg.smooth_weight - approach_pen * 2.0f - wobble_pen * cfg.wobble_weight
                - (died ? cfg.crash_penalty : 0.0f);
    if (finished) {
        float rate = progress * PW / (float)used;
        if (rate > 0.0f) score += rate * (float)(MAX_STEPS - used);
        score += cfg.finish_bonus * (1.0f - (float)used / (float)MAX_STEPS);
        float avg_v = forward_m / std::max(1e-3f, (float)used * DT);
        score += cfg.speed_bonus * std::clamp(avg_v / cfg.max_linear_speed_mps, 0.0f, 1.0f);
    }
    st.score = score; st.crashed = died; st.finished = finished; st.steps = used; st.forward_m = forward_m;
    st.time_s = used * DT;
    if (nq) { st.mean_abs_center = (float)(sum_c / nq); st.rms_center = (float)std::sqrt(sum_c2 / nq); st.mean_abs_heading = (float)(sum_h / nq); }
    return st;
}

// ------------------------------------------------------------------- config as "key value" text
// One parser for everything (the JS side writes the same keys). Sensors: "sensor_<i>_<field> value",
// and the upstream style "sensor_<i>_pose x y deg".
struct KeyF { const char* k; float Config::*p; };
struct KeyI { const char* k; int Config::*p; };

inline bool ApplyKey(Config& c, const std::string& key, const float* v, int nv) {
    static const KeyF F[] = {
        {"dt", &Config::dt}, {"track_width_m", &Config::track_width_m}, {"ctrl_track_width_m", &Config::ctrl_track_width_m},
        {"wheel_track_geom_m", &Config::wheel_track_geom_m}, {"wheel_radius_m", &Config::wheel_radius_m},
        {"max_linear_speed_mps", &Config::max_linear_speed_mps}, {"chassis_length_m", &Config::chassis_length_m},
        {"chassis_width_m", &Config::chassis_width_m}, {"chassis_front_m", &Config::chassis_front_m}, {"chassis_rear_m", &Config::chassis_rear_m},
        {"wheel_offset_from_center_m", &Config::wheel_offset_from_center_m}, {"wheel_length_m", &Config::wheel_length_m},
        {"wheel_width_m", &Config::wheel_width_m}, {"wheel_base_m", &Config::wheel_base_m},
        {"tof_glitch_threshold_m", &Config::tof_glitch_threshold_m}, {"tof_glitch_max_hold_s", &Config::tof_glitch_max_hold_s},
        {"tof_latency_s", &Config::tof_latency_s}, {"tof_latency_jitter_s", &Config::tof_latency_jitter_s},
        {"motor_tau_s", &Config::motor_tau_s}, {"motor_max_accel_mps2", &Config::motor_max_accel_mps2},
        {"wheel_slip_coefficient", &Config::wheel_slip_coefficient}, {"encoder_noise_base", &Config::encoder_noise_base},
        {"encoder_noise_speed_coeff", &Config::encoder_noise_speed_coeff}, {"speed_scale", &Config::speed_scale},
        {"battery_v", &Config::battery_v}, {"motor_mismatch_std", &Config::motor_mismatch_std}, {"enc_ticks_per_rev", &Config::enc_ticks_per_rev},
        {"pwm_limit", &Config::pwm_limit}, {"pwm_max", &Config::pwm_max}, {"wheel_pid_period_ms", &Config::wheel_pid_period_ms},
        {"wheel_kp", &Config::wheel_kp}, {"wheel_ki", &Config::wheel_ki}, {"wheel_kd", &Config::wheel_kd},
        {"wheel_ff_static", &Config::wheel_ff_static}, {"wheel_ff_gain", &Config::wheel_ff_gain},
        {"motor_k_l", &Config::motor_k_l}, {"motor_k_r", &Config::motor_k_r}, {"motor_static_l", &Config::motor_static_l},
        {"motor_static_r", &Config::motor_static_r}, {"motor_tau_l_s", &Config::motor_tau_l_s}, {"motor_tau_r_s", &Config::motor_tau_r_s},
        {"motor_nominal_v", &Config::motor_nominal_v}, {"motor_breakaway_l", &Config::motor_breakaway_l}, {"motor_breakaway_r", &Config::motor_breakaway_r},
        {"sensor_angle_bias_std_deg", &Config::sensor_angle_bias_std_deg}, {"wheel_scale_bias_std", &Config::wheel_scale_bias_std},
        {"max_speed_bias_std", &Config::max_speed_bias_std}, {"wheel_scale_l", &Config::wheel_scale_l}, {"wheel_scale_r", &Config::wheel_scale_r},
        {"emerg_front_m", &Config::emerg_front_m}, {"emerg_side_m", &Config::emerg_side_m}, {"emerg_stop_s", &Config::emerg_stop_s},
        {"emerg_back_speed", &Config::emerg_back_speed}, {"emerg_turn_frac", &Config::emerg_turn_frac}, {"emerg_back_m", &Config::emerg_back_m},
        {"emerg_turn_max_rad", &Config::emerg_turn_max_rad}, {"emerg_max_s", &Config::emerg_max_s},
        {"steer_tau_s", &Config::steer_tau_s}, {"steer_turn_accel", &Config::steer_turn_accel}, {"steer_lin_accel", &Config::steer_lin_accel},
        {"steer_max_step", &Config::steer_max_step}, {"kp", &Config::kp}, {"ki", &Config::ki}, {"kd", &Config::kd},
        {"base_speed_frac", &Config::base_speed_frac}, {"pid_base_speed_frac", &Config::base_speed_frac},
        {"pid_error_clip_m", &Config::pid_error_clip_m}, {"pid_integral_limit", &Config::pid_integral_limit},
        {"pid_d_filter_alpha", &Config::pid_d_filter_alpha}, {"pid_slowdown_frac", &Config::pid_slowdown_frac},
        {"follow_dist_m", &Config::follow_dist_m}, {"wall_noise_std_m", &Config::wall_noise_std_m}, {"max_episode_s", &Config::max_episode_s},
        {"spawn_margin_m", &Config::spawn_margin_m}, {"spawn_heading_min_frac", &Config::spawn_heading_min_frac},
        {"spawn_heading_random_deg", &Config::spawn_heading_random_deg}, {"spawn_lateral_random_frac", &Config::spawn_lateral_random_frac},
        {"parallel_weight", &Config::parallel_weight}, {"center_weight", &Config::center_weight}, {"heading_sigma_deg", &Config::heading_sigma_deg},
        {"center_sigma_frac", &Config::center_sigma_frac}, {"crash_penalty", &Config::crash_penalty}, {"wobble_weight", &Config::wobble_weight},
        {"wobble_omega_deadband", &Config::wobble_omega_deadband}, {"smooth_weight", &Config::smooth_weight},
        {"approach_deadband_mps", &Config::approach_deadband_mps}, {"finish_bonus", &Config::finish_bonus}, {"speed_bonus", &Config::speed_bonus},
    };
    static const KeyI I[] = {
        {"four_wheel", &Config::four_wheel}, {"n_sensors", &Config::n_sensors}, {"tof_glitch_max_hold_steps", &Config::tof_glitch_max_hold_steps},
        {"tof_delay_steps", &Config::tof_delay_steps}, {"encoder_model", &Config::encoder_model}, {"wheel_model", &Config::wheel_model},
        {"sim_substeps", &Config::sim_substeps}, {"emerg_on", &Config::emerg_on}, {"steer_smooth", &Config::steer_smooth},
        {"pid_mode", &Config::pid_mode}, {"follow_side", &Config::follow_side}, {"max_episode_steps", &Config::max_episode_steps},
    };
    if (nv < 1) return false;
    for (const auto& e : F) if (key == e.k) { c.*(e.p) = v[0]; return true; }
    for (const auto& e : I) if (key == e.k) { c.*(e.p) = (int)std::lround(v[0]); return true; }
    if (key.compare(0, 7, "sensor_") == 0 && key.size() > 9 && key[7] >= '0' && key[7] < '0' + MAXS && key[8] == '_') {
        Sensor& s = c.sensors[key[7] - '0'];
        std::string f = key.substr(9);
        if (f == "pose" && nv >= 3) { s.x_m = v[0]; s.y_m = v[1]; s.angle_deg = v[2]; return true; }
        if (f == "model") { s.model = (int)std::lround(v[0]); return true; }
        if (f == "rays") { s.rays = (int)std::lround(v[0]); return true; }
        if (f == "x_m") { s.x_m = v[0]; return true; }
        if (f == "y_m") { s.y_m = v[0]; return true; }
        if (f == "angle_deg") { s.angle_deg = v[0]; return true; }
        if (f == "half_angle_rad") { s.half_angle_rad = v[0]; return true; }
        if (f == "max_range_m") { s.max_range_m = v[0]; return true; }
        if (f == "min_range_m") { s.min_range_m = v[0]; return true; }
        if (f == "noise_std_m") { s.noise_std_m = v[0]; return true; }
        if (f == "noise_pct") { s.noise_pct = v[0]; return true; }
        if (f == "dropout") { s.dropout = v[0]; return true; }
        if (f == "bias_m") { s.bias_m = v[0]; return true; }
        if (f == "max_incidence_deg") { s.max_incidence_deg = v[0]; return true; }
    }
    return false;
}

// Parses "key v [v v]" lines ('#' comments). Returns the number of unknown keys.
inline int ParseConfig(Config& c, const char* text) {
    int unknown = 0;
    const char* p = text;
    while (*p) {
        const char* e = p;
        while (*e && *e != '\n') ++e;
        std::string line(p, e);
        p = *e ? e + 1 : e;
        size_t h = line.find('#');
        if (h != std::string::npos) line.resize(h);
        char key[96]; float v[4]; int nv = 0;
        const char* q = line.c_str();
        int consumed = 0;
        if (std::sscanf(q, "%95s%n", key, &consumed) != 1) continue;
        q += consumed;
        while (nv < 4 && std::sscanf(q, "%f%n", &v[nv], &consumed) == 1) { q += consumed; ++nv; }
        if (!ApplyKey(c, key, v, nv)) ++unknown;
    }
    c.n_sensors = std::clamp(c.n_sensors, 1, MAXS);
    if (c.dt < 0.001f) c.dt = 0.001f;
    return unknown;
}

}  // namespace rt
