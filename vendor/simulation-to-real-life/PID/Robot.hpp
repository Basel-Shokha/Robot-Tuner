#pragma once
#include "Types.hpp"
#include "Raycast.hpp"
#include "MotorModel.hpp"
#include "../firmware/robot_fw/emergency.h"
#include "../firmware/robot_fw/steer_smooth.h"
#include "../firmware/robot_fw/reflex.h"
#include "PidController.hpp"
#include <vector>
#include <algorithm>
#include <cmath>

// SIMULATION_DT (the control step, s) is defined in SimConfig.hpp and set from control_period_s.

enum class RobotState {
    NAVIGATING,
    EMERG_STOP,     // emergency: standing still (firmware/robot_fw/emergency.h)
    EMERG_REVERSE   // emergency: reversing, nose turning away from the closer wall
};

struct Robot {
    Vec2 pos{0.0f, 0.0f};
    float theta{0.0f};
    float v_l{0.0f};
    float v_r{0.0f};
    float prev_v_l{0.0f};
    // Saturation counters (experiments, docs/results/speed_sweep): navigating ticks, ticks where the controller's
    // requested turn was cut by a wheel ceiling, ticks where a wheel PID sat at the PWM limit.
    long n_nav{0}, n_sat_cmd{0}, n_sat_pwm{0};
    // JOB 11 fixed-clock sensor schedule (cfg.fixed_clock; per episode)
    float fc_t{0.0f}, fc_phase[3]{0, 0, 0}, fc_T[3]{0.025f, 0.025f, 0.025f}, fc_held[3]{0, 0, 0};
    long fc_last_k[3]{-2, -2, -2};
    bool fc_new[3]{true, true, true};
    bool ctrl_sat{false};
    float prev_v_r{0.0f};
    float dists[3]{4.0f, 4.0f, 4.0f};
    bool alive{true};
    float fitness{0.0f};

    float emergency_stop_threshold{0.12f};
    emerg::Logic emergency;  // shared with the firmware
    reflex::Logic reflex;    // side-wall reflex (reflex.h), benchmark only, off unless reflex_on
    reflex::Slow slow;       // slow down near a side wall (reflex.h), benchmark only, off unless slow_on
    smooth::Filter steer;    // output smoothing, shared with the firmware
    bool performed_scan{false};
    float scan_trigger_dist{0.0f};
    bool finished{false};

    RobotState state{RobotState::NAVIGATING};
    float target_angle{0.0f};
    float start_heading{0.0f};
    // Rotation accumulated since the scan started, signed, unbounded. Absolute heading cannot be
    // compared against a target on hardware, where the IMU wraps at +/-pi; this cannot wrap.
    float scan_rotation_accum{0.0f};
    float left_recorded_dist{0.0f};
    float right_recorded_dist{0.0f};

    float last_valid_dists[3]{4.0f, 4.0f, 4.0f};
    int glitch_hold_count[3]{0, 0, 0};
    float last_raw_center_dist{4.0f};
    float last_eff_v_l{0.0f};
    float last_eff_v_r{0.0f};

    // Real wheel speeds (motor output before slip and wheel scale); equal to the command unless
    // motor_tau_s / motor_max_accel_mps2 model a lag (tasks.txt C2).
    float wheel_l{0.0f};
    float wheel_r{0.0f};
    // This tick's encoder readings (m/s) for the motion of the last tick: what the controllers see.
    float enc_l{0.0f};
    float enc_r{0.0f};
    // Raw (unfiltered, possibly delayed) ToF readings of this tick, for logging / equivalence tests.
    float raw_dists[3]{4.0f, 4.0f, 4.0f};
    static constexpr int TOF_HIST = 4;
    float tof_hist[TOF_HIST][3]{};
    int tof_hist_n{0};
    bool scan_motion{false};  // the motion just executed was part of a scan turn
    float resume_left{0.0f};  // s of straight driving left after a scan
    // Recent poses (index 0 = now), for the ToF reading age (tof_latency_s).
    static constexpr int POSE_HIST = 8;
    Vec2 pose_hist_p[POSE_HIST]{};
    float pose_hist_th[POSE_HIST]{};
    int pose_hist_n{0};
    // wheel_model pid: the inner loop (wheel PID + motors + encoders), and the per-run motor gains.
    DriveSim drive;
    // Optional log of every wheel-PID tick (tools/equiv): set pid_trace / pid_trace_max, reset pid_trace_n.
    DriveSim::PidTrace* pid_trace{nullptr};
    int pid_trace_n{0}, pid_trace_max{0};
    float pack_scale{1.0f};
    float motor_mismatch[2]{1.0f, 1.0f};
    // Adapter so DriveSim can integrate this robot's pose in sub-steps.
    struct PoseRef {
        Vec2& pos; float& theta;
        void Integrate(float vl, float vr, float track, float h) {
            float v = 0.5f * (vl + vr), w = (vr - vl) / track;
            pos.x += v * std::cos(theta) * h;
            pos.y += v * std::sin(theta) * h;
            theta += w * h;
        }
    };

    // Per-run systematic error: fixed for a whole episode, so the robot must cope with a machine
    // that is consistently wrong rather than noisily wrong. All neutral until SampleHardwareBias.
    float sensor_angle_bias_deg[3]{0.0f, 0.0f, 0.0f};
    float wheel_scale_l{1.0f};
    float wheel_scale_r{1.0f};

    PidBrain brain;

    void Reset(Vec2 start_pos, float start_heading_rad = 0.0f, float tof_max_range = 4.0f) {
        pos = start_pos;
        theta = start_heading_rad;
        v_l = 0.0f;
        v_r = 0.0f;
        prev_v_l = 0.0f;
        prev_v_r = 0.0f;
        alive = true;
        fitness = 0.0f;
        state = RobotState::NAVIGATING;
        n_nav = n_sat_cmd = n_sat_pwm = 0; ctrl_sat = false;
        fc_t = 0.0f; for (int i = 0; i < 3; ++i) { fc_last_k[i] = -2; fc_new[i] = true; }
        performed_scan = false;
        emergency.Reset();
        reflex.Reset();
        slow.Reset();
        steer.Reset();
        scan_trigger_dist = 0.0f;
        finished = false;
        last_raw_center_dist = tof_max_range;
        last_eff_v_l = 0.0f;
        last_eff_v_r = 0.0f;
        for (float& d : dists) d = tof_max_range;
        for (float& d : last_valid_dists) d = tof_max_range;
        for (int& c : glitch_hold_count) c = 0;
        scan_rotation_accum = 0.0f;
        for (float& b : sensor_angle_bias_deg) b = 0.0f;
        wheel_scale_l = 1.0f;
        wheel_scale_r = 1.0f;
        wheel_l = wheel_r = 0.0f;
        enc_l = enc_r = 0.0f;
        for (float& d : raw_dists) d = tof_max_range;
        tof_hist_n = 0;
        scan_motion = false;
        resume_left = 0.0f;
        pose_hist_n = 0;
        drive.Reset();
        pack_scale = 1.0f;
        motor_mismatch[0] = motor_mismatch[1] = 1.0f;
        brain.ResetState();
    }

    // Draw this run's systematic error. Call after Reset, with the episode's rng, so each episode
    // gets a different "build" of the robot. With all three std values at 0 this is a no-op.
    void SampleHardwareBias(const SimConfig& cfg, std::mt19937& rng) {
        if (cfg.fixed_clock) {  // JOB 11: each sensor's own period and phase (drawn only in this mode)
            std::normal_distribution<float> tol(0.0f, cfg.fc_clock_tol);
            std::uniform_real_distribution<float> ph(0.0f, 1.0f);
            for (int i = 0; i < 3; ++i) { fc_T[i] = cfg.tof_period_s * (1.0f + tol(rng)); fc_phase[i] = ph(rng) * fc_T[i]; }
        }
        if (cfg.sensor_angle_bias_std_deg > 0.0f) {
            std::normal_distribution<float> d(0.0f, cfg.sensor_angle_bias_std_deg);
            for (int i = 0; i < 3; ++i) sensor_angle_bias_deg[i] = d(rng);
        }
        if (cfg.wheel_scale_bias_std > 0.0f) {
            std::normal_distribution<float> d(0.0f, cfg.wheel_scale_bias_std);
            wheel_scale_l = std::clamp(1.0f + d(rng), 0.5f, 1.5f);
            wheel_scale_r = std::clamp(1.0f + d(rng), 0.5f, 1.5f);
        }
        if (cfg.max_speed_bias_std > 0.0f) {
            // A weaker pack scales both wheels together, so it folds into the same factors.
            std::normal_distribution<float> d(0.0f, cfg.max_speed_bias_std);
            float pack = std::clamp(1.0f + d(rng), 0.5f, 1.2f);
            if (cfg.wheel_model == 1) {
                pack_scale = pack;  // a weak pack slows the MOTORS; the wheel PID can make up for it
            } else {
                wheel_scale_l *= pack;
                wheel_scale_r *= pack;
            }
        }
        if (cfg.wheel_model == 1 && cfg.motor_mismatch_std > 0.0f) {  // drawn last: simple mode unchanged
            std::normal_distribution<float> d(0.0f, cfg.motor_mismatch_std);
            motor_mismatch[0] = std::clamp(1.0f + d(rng), 0.5f, 1.5f);
            motor_mismatch[1] = std::clamp(1.0f + d(rng), 0.5f, 1.5f);
        }
    }

    // Robot origin (axle midpoint) for a track spawn point. With spawn_at_body_center the spawn
    // point is the centre of the body outline, so a robot whose axle sits near the rear is not
    // spawned with its nose in the wall.
    static Vec2 SpawnAxle(const SimConfig& cfg, Vec2 spawn, float heading) {
        if (!cfg.spawn_at_body_center) return spawn;
        float cx = 0.5f * (ChassisFront(cfg) - ChassisRear(cfg));  // body centre, ahead of the axle
        return {spawn.x - std::cos(heading) * cx, spawn.y - std::sin(heading) * cx};
    }

    static float ChassisFront(const SimConfig& cfg) {
        return cfg.chassis_front_m > 0.0f ? cfg.chassis_front_m : 0.6f * cfg.chassis_length_m;
    }
    static float ChassisRear(const SimConfig& cfg) {
        return cfg.chassis_rear_m > 0.0f ? cfg.chassis_rear_m : 0.4f * cfg.chassis_length_m;
    }

    // ToF latency (tasks.txt C4): returns the reading taken tof_delay_steps ticks ago.
    float DelayedToF(int i, float fresh, const SimConfig& cfg) {
        int delay = std::clamp(cfg.tof_delay_steps, 0, TOF_HIST - 1);
        if (delay == 0) return fresh;
        if (i == 0) {  // shift once per tick
            for (int k = TOF_HIST - 1; k > 0; --k)
                for (int j = 0; j < 3; ++j) tof_hist[k][j] = tof_hist[k - 1][j];
            tof_hist_n = std::min(tof_hist_n + 1, TOF_HIST);
        }
        tof_hist[0][i] = fresh;
        int k = std::min(delay, tof_hist_n - 1);
        return tof_hist[k][i];
    }

    // Pose `age` seconds ago, interpolated between the stored control steps (clamped to the oldest one).
    void PastPose(float age, Vec2& p, float& th) const {
        if (pose_hist_n <= 1 || age <= 0.0f) { p = pos; th = theta; return; }
        float k = age / SIMULATION_DT;
        int a = std::min((int)k, pose_hist_n - 1);
        int b = std::min(a + 1, pose_hist_n - 1);
        float f = std::clamp(k - (float)a, 0.0f, 1.0f);
        if (a == b) f = 0.0f;
        p = {pose_hist_p[a].x + f * (pose_hist_p[b].x - pose_hist_p[a].x), pose_hist_p[a].y + f * (pose_hist_p[b].y - pose_hist_p[a].y)};
        float dth = pose_hist_th[b] - pose_hist_th[a];
        while (dth > PI_F) dth -= 2.0f * PI_F;
        while (dth < -PI_F) dth += 2.0f * PI_F;
        th = pose_hist_th[a] + f * dth;
    }

    // First-order lag + acceleration limit of one wheel (tasks.txt C2). Off = instant.
    static float MotorResponse(float wheel, float cmd, const SimConfig& cfg, float dt) {
        float next = cfg.motor_tau_s > 0.0f ? wheel + (cmd - wheel) * (1.0f - std::exp(-dt / cfg.motor_tau_s)) : cmd;
        if (cfg.motor_max_accel_mps2 > 0.0f) {
            float dv = cfg.motor_max_accel_mps2 * dt;
            next = std::clamp(next, wheel - dv, wheel + dv);
        }
        return next;
    }

    // Segment-segment intersection using the same parametric cross-product form as CastSingleRay.
    static bool SegmentsIntersect(Vec2 a1, Vec2 a2, Vec2 b1, Vec2 b2) {
        float rx = a2.x - a1.x, ry = a2.y - a1.y;
        float sx = b2.x - b1.x, sy = b2.y - b1.y;
        float cross = rx * sy - ry * sx;
        if (std::abs(cross) < 1e-9f) return false;
        float qx = b1.x - a1.x, qy = b1.y - a1.y;
        float t = (qx * sy - qy * sx) / cross;
        float u = (qx * ry - qy * rx) / cross;
        return t >= 0.0f && t <= 1.0f && u >= 0.0f && u <= 1.0f;
    }

    // Rectangular body outline around the origin (axle midpoint): local x spans
    // [-chassis_rear_m, +chassis_front_m], y spans [-W/2, W/2].
    void ChassisCorners(const SimConfig& cfg, Vec2 corners[4]) const {
        float rear = -ChassisRear(cfg);
        float front = ChassisFront(cfg);
        float half_w = 0.5f * cfg.chassis_width_m;
        const float local[4][2] = {{front, -half_w}, {front, half_w}, {rear, half_w}, {rear, -half_w}};

        float c = std::cos(theta), s = std::sin(theta);
        for (int i = 0; i < 4; ++i) {
            float dx = local[i][0], dy = local[i][1];
            corners[i] = {pos.x + c * dx - s * dy, pos.y + s * dx + c * dy};
        }
    }

    // True if any chassis edge intersects the segment (shared by wall collision and finish detection).
    static bool ChassisTouchesSegment(const Vec2 corners[4], const Wall& seg) {
        for (int i = 0; i < 4; ++i) {
            if (SegmentsIntersect(corners[i], corners[(i + 1) % 4], seg.p1, seg.p2)) return true;
        }
        return false;
    }

    // The two drive wheels, aligned with the visualizer: centred wheel_offset_from_center_m behind the
    // robot origin and +-track_width_m/2 to the sides, wheel_length_m long and wheel_width_m wide.
    // On the real robot the wheels stick out past the chassis (12 cm between wheel centres vs an 8 cm
    // chassis), so they are usually the first thing to touch a wall.
    void WheelCorners(const SimConfig& cfg, int side, Vec2 corners[4]) const {
        float cx = -cfg.wheel_offset_from_center_m;
        float cy = (side == 0 ? 0.5f : -0.5f) * cfg.track_width_m;
        float hl = 0.5f * cfg.wheel_length_m, hw = 0.5f * cfg.wheel_width_m;
        const float local[4][2] = {{cx + hl, cy - hw}, {cx + hl, cy + hw}, {cx - hl, cy + hw}, {cx - hl, cy - hw}};
        float c = std::cos(theta), s = std::sin(theta);
        for (int i = 0; i < 4; ++i) {
            float dx = local[i][0], dy = local[i][1];
            corners[i] = {pos.x + c * dx - s * dy, pos.y + s * dx + c * dy};
        }
    }

    // Distance from the robot origin to the farthest point of the footprint (chassis or wheels):
    // the radius it sweeps when turning in place.
    static float TurnInPlaceRadius(const SimConfig& cfg) {
        float r = std::max(std::hypot(ChassisFront(cfg), 0.5f * cfg.chassis_width_m),
                           std::hypot(ChassisRear(cfg), 0.5f * cfg.chassis_width_m));
        float wx = std::abs(cfg.wheel_offset_from_center_m) + 0.5f * cfg.wheel_length_m;
        float wy = 0.5f * cfg.track_width_m + 0.5f * cfg.wheel_width_m;
        return std::max(r, std::hypot(wx, wy));
    }

    // Collision: the chassis OR either wheel touching any wall.
    bool ChassisHitsWall(const std::vector<Wall>& track, const SimConfig& cfg) const {
        Vec2 parts[3][4];
        ChassisCorners(cfg, parts[0]);
        WheelCorners(cfg, 0, parts[1]);
        WheelCorners(cfg, 1, parts[2]);
        for (const auto& w : track) {
            for (const auto& corners : parts) {
                if (ChassisTouchesSegment(corners, w)) return true;
            }
        }
        return false;
    }

    // finish: optional finish segment; when the chassis crosses it, finished is set (alive stays true).
    void Step(const std::vector<Wall>& track, const SimConfig& cfg, std::mt19937& rng, const Wall* finish = nullptr) {
        if (!alive) return;

        // Pose history for the ToF reading age: this step's pose is index 0.
        for (int k = POSE_HIST - 1; k > 0; --k) { pose_hist_p[k] = pose_hist_p[k - 1]; pose_hist_th[k] = pose_hist_th[k - 1]; }
        pose_hist_p[0] = pos; pose_hist_th[0] = theta;
        pose_hist_n = std::min(pose_hist_n + 1, POSE_HIST);

        for (int i = 0; i < 3; ++i) {
            SensorMount mount = cfg.sensors[i];
            mount.angle_deg += sensor_angle_bias_deg[i];  // this build's mounting error
            // The reading describes where the robot was tof_latency_s (+ jitter) ago, not where it is now.
            Vec2 sp = pos; float sth = theta;
            if (cfg.tof_latency_s > 0.0f) {
                float age = cfg.tof_latency_s;
                if (cfg.tof_latency_jitter_s > 0.0f) { std::uniform_real_distribution<float> j(0.0f, cfg.tof_latency_jitter_s); age += j(rng); }
                PastPose(age, sp, sth);
            }
            float raw_dist;
            if (cfg.fixed_clock) {
                // Newest sample usable at this tick (completed at phase + k T, read fc_read_s later). A new one is ranged
                // from the pose at its exposure centre (budget/2 before completion); an old one is held unchanged.
                long k = (long)std::floor((fc_t - fc_phase[i] - cfg.fc_read_s) / fc_T[i]);
                if (k < -1) k = -1;  // before the first sample: the one completing at the phase (as if just before start)
                fc_new[i] = k > fc_last_k[i];
                if (fc_new[i]) {
                    float age = fc_t - (fc_phase[i] + (float)k * fc_T[i] - 0.5f * cfg.tof_timing_budget_s);
                    PastPose(std::max(0.0f, age), sp, sth);
                    fc_held[i] = SampleToFSensorAtMount(sp, sth, mount, track, cfg, rng);
                    fc_last_k[i] = k;
                }
                raw_dist = fc_held[i];
            } else {
                raw_dist = DelayedToF(i, SampleToFSensorAtMount(sp, sth, mount, track, cfg, rng), cfg);
            }
            raw_dists[i] = raw_dist;
            if (i == 1) {
                last_raw_center_dist = raw_dist;
            }

            // Asymmetric glitch rejection, with a bounded hold. A single dropout reading reads far,
            // so we reject it -- but a real opening (corridor end, side branch) also reads far, and
            // holding it forever would hide exactly the openings the scan logic needs. After
            // tof_glitch_max_hold_steps consecutive high readings we accept that the world changed.
            if (raw_dist > last_valid_dists[i] && (raw_dist - last_valid_dists[i]) > cfg.tof_glitch_threshold_m
                && glitch_hold_count[i] < cfg.GlitchHoldSteps()) {
                dists[i] = last_valid_dists[i];
                ++glitch_hold_count[i];
            } else {
                dists[i] = raw_dist;
                last_valid_dists[i] = raw_dist;
                glitch_hold_count[i] = 0;
            }
        }

        if (ChassisHitsWall(track, cfg)) {
            alive = false;
            return;
        }

        prev_v_l = v_l;
        prev_v_r = v_r;

        // Encoders: the motion of the last tick, as the controllers see it (m/s). encoder_model 0
        // reports the command + noise (legacy); 1 reports the wheel's real speed (lag and per-run
        // wheel scale included, slip not) + noise, which is what a wheel encoder measures (C1).
        if (cfg.wheel_model == 1) {
            // Quantised encoder ticks of the simulated wheels, converted exactly like the firmware.
            drive.ReadControlEncoders(cfg, SIMULATION_DT, enc_l, enc_r);
            if (scan_motion) scan_rotation_accum += (enc_r - enc_l) / brain.hw.track_width * SIMULATION_DT;
        } else {
            float base_l = cfg.encoder_model == 1 ? wheel_l * wheel_scale_l : v_l;
            float base_r = cfg.encoder_model == 1 ? wheel_r * wheel_scale_r : v_r;
            float sigma_l = cfg.encoder_noise_base + cfg.encoder_noise_speed_coeff * std::abs(base_l);
            float sigma_r = cfg.encoder_noise_base + cfg.encoder_noise_speed_coeff * std::abs(base_r);
            enc_l = base_l;
            enc_r = base_r;
            if (sigma_l > 0.0f) { std::normal_distribution<float> n(0.0f, sigma_l); enc_l += n(rng); }
            if (sigma_r > 0.0f) { std::normal_distribution<float> n(0.0f, sigma_r); enc_r += n(rng); }
            // With real encoders the scan measures its rotation from them, like the firmware.
            if (cfg.encoder_model == 1 && scan_motion)
                scan_rotation_accum += (enc_r - enc_l) / brain.hw.track_width * SIMULATION_DT;
        }

        // Emergency manoeuvre (firmware/robot_fw/emergency.h, the SAME code as on the robot): stop, then
        // reverse while turning the nose away from the closer wall. Replaces the spin-in-place scan, which
        // needs a 313 mm wide corridor; the real ones are 200-250 mm.
        emergency.p.enabled = cfg.emerg_on; emergency.p.front_m = cfg.emerg_front_m; emergency.p.side_m = cfg.emerg_side_m;
        emergency.p.stop_s = cfg.emerg_stop_s; emergency.p.back_speed = cfg.emerg_back_speed;
        emergency.p.turn_frac = cfg.emerg_turn_frac; emergency.p.back_m = cfg.emerg_back_m;
        emergency.p.turn_max_rad = cfg.emerg_turn_max_rad; emergency.p.max_s = cfg.emerg_max_s;
        emergency.p.track_w = brain.hw.track_width;
        bool emerg_ended = false;
        bool emerg_active = emergency.Step(dists, enc_l, enc_r, SIMULATION_DT, v_l, v_r, emerg_ended);
        // Side-wall reflex (reflex.h; benchmark only, off unless reflex_on): halt, rotate 45 deg away, continue.
        reflex.p.enabled = cfg.reflex_on; reflex.p.trigger_m = cfg.reflex_trigger_m; reflex.p.halt_s = cfg.reflex_halt_s;
        reflex.p.turn_rad = cfg.reflex_turn_deg * PI_F / 180.0f; reflex.p.turn_speed = cfg.reflex_turn_speed;
        reflex.p.track_w = brain.hw.track_width;
        bool reflex_ended = false;
        bool reflex_active = !emerg_active && reflex.Step(dists, enc_l, enc_r, SIMULATION_DT, v_l, v_r, reflex_ended);
        if (emerg_ended || reflex_ended) {
            steer.Reset();  // the robot was reversing: restart the smoothing from standstill
            brain.ResetState();  // the error history from before the manoeuvre no longer applies
        }
        if (reflex_active) {
            state = RobotState::EMERG_STOP;  // not the controller driving (counts as a manoeuvre)
        } else if (emerg_active) {
            performed_scan = true;
            state = emergency.state == emerg::STOP ? RobotState::EMERG_STOP : RobotState::EMERG_REVERSE;
        } else {
            state = RobotState::NAVIGATING;

            // PID in control. It steers from the two side ToF readings only; the wheel encoders
            // are not inputs here, so encoder noise does not reach this controller (wheel slip and
            // the per-run wheel scale error still act on the motion below).
            brain.fc_new_reading = !cfg.fixed_clock || fc_new[brain.left_idx] || fc_new[brain.right_idx];
            brain.Control(dists, SIMULATION_DT, v_l, v_r);
            slow.enabled = cfg.slow_on; slow.slow_m = cfg.slow_m; slow.slow_frac = cfg.slow_frac; slow.keep_turn = cfg.slow_keep_turn;
            slow.Apply(dists, v_l, v_r);  // benchmark only (reflex.h)
            steer.p.enabled = cfg.steer_smooth; steer.p.turn_tau_s = cfg.steer_tau_s;
            steer.p.turn_accel = cfg.steer_turn_accel; steer.p.lin_accel = cfg.steer_lin_accel; steer.p.max_step = cfg.fixed_clock ? cfg.steer_max_step * SIMULATION_DT / 0.025f : cfg.steer_max_step;
            steer.Apply(v_l, v_r, SIMULATION_DT);
            emerg::NoReverse(v_l, v_r);  // never drive backwards (emergency.h)
            ++n_nav; if (brain.sat) ++n_sat_cmd;
        }

        // Differential kinematic integration with slip
        if (cfg.wheel_model == 1) {
            // Inner loop: firmware wheel PID -> PWM -> DC motor, in sub-steps; pose integrated inside.
            // speed_scale scales the wheel targets after the controller, exactly like the firmware's speed_scale.
            float targets[2] = {v_l * cfg.speed_scale, v_r * cfg.speed_scale};
            float base_gain = cfg.battery_v / cfg.motor_nominal_v * pack_scale;
            float gain[2] = {base_gain * motor_mismatch[0], base_gain * motor_mismatch[1]};
            float scale[2] = {wheel_scale_l, wheel_scale_r};
            float ground[2];
            PoseRef pr{pos, theta};
            drive.Step(cfg, SIMULATION_DT, targets, gain, scale, pr, ground, pid_trace, &pid_trace_n, pid_trace_max);
            if (state == RobotState::NAVIGATING && (std::abs(drive.pwm[0]) >= (int)cfg.pwm_limit || std::abs(drive.pwm[1]) >= (int)cfg.pwm_limit)) ++n_sat_pwm;
            wheel_l = drive.w[0]; wheel_r = drive.w[1];
            last_eff_v_l = ground[0]; last_eff_v_r = ground[1];
            // The scan's rotation comes from the encoders (read at the start of the next Step).
        } else {
            float prev_wheel_l = wheel_l, prev_wheel_r = wheel_r;
            wheel_l = MotorResponse(wheel_l, v_l * cfg.speed_scale, cfg, SIMULATION_DT);
            wheel_r = MotorResponse(wheel_r, v_r * cfg.speed_scale, cfg, SIMULATION_DT);
            float accel_l = std::abs(wheel_l - prev_wheel_l) / SIMULATION_DT;
            float accel_r = std::abs(wheel_r - prev_wheel_r) / SIMULATION_DT;
            float slip_factor_l = 1.0f - std::min(0.15f, cfg.wheel_slip_coefficient * accel_l);
            float slip_factor_r = 1.0f - std::min(0.15f, cfg.wheel_slip_coefficient * accel_r);

            // The wheels do not deliver what was commanded: mismatched wheels/motors and a sagging
            // pack scale each side. The encoder inputs above still report the commanded value, which is
            // the point -- the robot cannot see this error, it can only be robust to it.
            float v_eff_l = wheel_l * slip_factor_l * wheel_scale_l;
            float v_eff_r = wheel_r * slip_factor_r * wheel_scale_r;
            last_eff_v_l = v_eff_l;
            last_eff_v_r = v_eff_r;

            float v = (v_eff_l + v_eff_r) * 0.5f;
            float omega = (v_eff_r - v_eff_l) / cfg.track_width_m;  // physics: the real track width

            pos.x += v * std::cos(theta) * SIMULATION_DT;
            pos.y += v * std::sin(theta) * SIMULATION_DT;
            theta += omega * SIMULATION_DT;
            if (cfg.encoder_model != 1 && state != RobotState::NAVIGATING) scan_rotation_accum += omega * SIMULATION_DT;
        }
        scan_motion = state != RobotState::NAVIGATING;
        // Keep theta in (-pi, pi] like a real IMU, so sim and firmware agree on what a heading is.
        while (theta > PI_F) theta -= 2.0f * PI_F;
        while (theta <= -PI_F) theta += 2.0f * PI_F;
        fc_t += SIMULATION_DT;

        if (finish && !finished) {
            Vec2 corners[4];
            ChassisCorners(cfg, corners);
            if (ChassisTouchesSegment(corners, *finish)) finished = true;
        }
    }
};