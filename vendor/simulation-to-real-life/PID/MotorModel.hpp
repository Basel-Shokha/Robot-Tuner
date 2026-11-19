#pragma once
// MotorModel.hpp - the real robot's inner loop, for wheel_model pid (error_params.txt).
//
//   outer controller (NN / PID / LineFit, 20 Hz) -> wheel speed targets (m/s)
//     -> wheel PID (firmware code, fw::WheelPid, every wheel_pid_period_ms) -> PWM
//     -> DC motor model (below)                    -> wheel speed -> encoder ticks (quantised)
//
// The wheel PID is NOT a copy: it is firmware/robot_fw/wheel_pid.h, the file the ESP32 runs.
// Identical in NN/, PID/, LineFit/.
//
// Motor model, per wheel, first order:
//   w_ss = sign(pwm) * k * (|pwm| - static_pwm) * gain     if |pwm| > static_pwm, else 0
//   but a wheel AT REST only starts when |pwm| > breakaway_pwm (stiction; measured: starts at ~20, then the
//   line extrapolates to ~3.5).
//   dw/dt = (w_ss - w) / tau
// w is the wheel surface speed (m/s) for the NOMINAL wheel diameter, i.e. what the encoder measures.
// gain = (battery_v / motor_nominal_v) * per-run pack factor * per-run motor mismatch (the PID can correct
// these as long as it is not clamped). The ground speed is w * slip * wheel_scale (diameter / calibration
// error and slip, which the encoder cannot see).
#include "SimConfig.hpp"
#include "../firmware/robot_fw/wheel_pid.h"
#include <cmath>
#include <algorithm>

struct DriveSim {
    float w[2] = {0.0f, 0.0f};        // wheel surface speed, m/s (0 = left, 1 = right)
    double dist[2] = {0.0, 0.0};      // wheel surface distance from rotation, m (what the encoder counts)
    long long tick_prev_pid[2] = {0, 0};
    long long tick_prev_ctrl[2] = {0, 0};
    fw::WheelPid pid[2];
    int pwm[2] = {0, 0};
    float pid_timer = 0.0f;           // time until the next wheel PID tick, s
    bool raw_mode = false;            // calibration: drive raw_pwm directly, no PID (firmware raw_mode)
    int raw_pwm[2] = {0, 0};

    void Reset() { *this = DriveSim(); }

    static float MetersPerTick(const SimConfig& c) {
        return fw::MetersPerTick(2.0f * c.wheel_radius_m, c.enc_ticks_per_rev, 1.0f);
    }
    long long Ticks(int i, const SimConfig& c) const { return (long long)std::floor(dist[i] / MetersPerTick(c)); }

    // Encoder speeds of the last control tick (m/s), as the firmware computes them: tick delta * m/tick / dt.
    void ReadControlEncoders(const SimConfig& c, float dt, float& enc_l, float& enc_r) {
        long long t0 = Ticks(0, c), t1 = Ticks(1, c);
        float mpt = MetersPerTick(c);
        enc_l = (float)(t0 - tick_prev_ctrl[0]) * mpt / dt;
        enc_r = (float)(t1 - tick_prev_ctrl[1]) * mpt / dt;
        tick_prev_ctrl[0] = t0; tick_prev_ctrl[1] = t1;
    }

    static fw::WheelPidGains Gains(const SimConfig& c) {
        fw::WheelPidGains g;
        g.kp = c.wheel_kp; g.ki = c.wheel_ki; g.kd = c.wheel_kd;
        g.ff_static = c.wheel_ff_static; g.ff_gain = c.wheel_ff_gain;
        g.pwm_limit = c.pwm_limit; g.pwm_max = c.pwm_max;
        return g;
    }

    // Steady-state wheel speed for a PWM (signed), one wheel.
    static float SteadySpeed(const SimConfig& c, int i, int pwm_cmd, float gain, bool at_rest = false) {
        float k = i == 0 ? c.motor_k_l : c.motor_k_r;
        float st = i == 0 ? c.motor_static_l : c.motor_static_r;
        float brk = i == 0 ? c.motor_breakaway_l : c.motor_breakaway_r;
        float a = std::abs((float)pwm_cmd);
        if (a <= st || (at_rest && a <= brk)) return 0.0f;
        return (pwm_cmd > 0 ? 1.0f : -1.0f) * k * (a - st) * gain;
    }

    // One PID tick: measure (tick delta since the last PID tick = ticks per PID period) and set the PWM.
    // target_tp: wheel targets in ticks per PID period. Exposed so the equivalence test can record it.
    struct PidTrace { float target[2], meas[2]; int pwm[2]; };
    PidTrace PidTick(const SimConfig& c, const float target_tp[2]) {
        fw::WheelPidGains g = Gains(c);
        PidTrace tr{};
        for (int i = 0; i < 2; ++i) {
            long long t = Ticks(i, c);
            float meas = (float)(t - tick_prev_pid[i]);
            tick_prev_pid[i] = t;
            if (raw_mode) { pid[i].Reset(); pwm[i] = raw_pwm[i]; }
            else pwm[i] = pid[i].Step(g, true, target_tp[i], meas);
            tr.target[i] = target_tp[i]; tr.meas[i] = meas; tr.pwm[i] = pwm[i];
        }
        return tr;
    }

    // Advance one control tick (dt) in sim_substeps sub-steps: PID ticks at their own rate, motor, encoders,
    // slip and the robot pose. targets: wheel speeds in m/s from the outer controller.
    // Returns the mean ground speeds over the tick (for telemetry). trace/n_trace: optional PID log.
    template <class Pose>
    void Step(const SimConfig& c, float dt, const float targets_mps[2], const float gain[2], const float wheel_scale[2],
              Pose& pose, float ground_mean[2], PidTrace* trace = nullptr, int* n_trace = nullptr, int max_trace = 0) {
        int n = std::max(1, c.sim_substeps);
        float h = dt / n;
        float mpt = MetersPerTick(c);
        float period = std::max(0.001f, c.wheel_pid_period_ms / 1000.0f);
        float tgt[2] = {fw::MpsToTp(targets_mps[0], mpt, period), fw::MpsToTp(targets_mps[1], mpt, period)};
        float tau[2] = {std::max(1e-4f, c.motor_tau_l_s), std::max(1e-4f, c.motor_tau_r_s)};
        ground_mean[0] = ground_mean[1] = 0.0f;
        for (int s = 0; s < n; ++s) {
            if (pid_timer <= 1e-6f) {
                PidTrace tr = PidTick(c, tgt);
                if (trace && n_trace && *n_trace < max_trace) trace[(*n_trace)++] = tr;
                pid_timer += period;
            }
            pid_timer -= h;
            float vg[2];
            for (int i = 0; i < 2; ++i) {
                float prev = w[i];
                float ss = SteadySpeed(c, i, pwm[i], gain[i], std::abs(w[i]) < 1e-4f);
                w[i] += (ss - w[i]) * (1.0f - std::exp(-h / tau[i]));
                dist[i] += (double)w[i] * h;
                // Same slip law as the simple model, on the real wheel acceleration.
                float slip = 1.0f - std::min(0.15f, c.wheel_slip_coefficient * std::abs(w[i] - prev) / h);
                vg[i] = w[i] * slip * wheel_scale[i];
                ground_mean[i] += vg[i] / n;
            }
            pose.Integrate(vg[0], vg[1], c.track_width_m, h);
        }
    }
};
