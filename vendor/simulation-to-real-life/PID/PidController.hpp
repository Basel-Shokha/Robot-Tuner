#pragma once
#include "Types.hpp"
#include "SimConfig.hpp"
#include <algorithm>
#include <random>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

// PID wall-following controller: the drop-in replacement for the NN project's NeuralBrain.
//
// The idea in one line: the two side sensors (+-45 deg) should read the same distance. If they do,
// the robot is centred AND parallel to the walls. Any difference is the error, and the PID turns
// the robot to drive that difference back to zero.
//
//   err  = left - right        (+ means more room on the left: we are too far right, or pointing right)
//   P    = Kp * err            react to where we are NOW
//   I    = Ki * integral(err)  react to an error that has PERSISTED (e.g. one wheel weaker than the other)
//   D    = Kd * d(err)/dt      react to how FAST the error is changing (brake before overshooting)
//   turn = P + I + D           wheel speed difference: v_r = base + turn, v_l = base - turn
//
// "Training" chooses Kp, Ki, Kd (and, optionally, the cruise speed) with the very same genetic
// algorithm and reward the NN uses, so the two controllers are compared on equal terms.
//
// The controller does not use the wheel encoders: the error comes only from the ToF sensors.

struct HardwareParams {
    float track_width = 0.120f;
    float wheel_radius = 0.032f;
    float max_linear_speed = 0.45f;
    float left_angle_deg = -45.0f;
    float center_angle_deg = 0.0f;
    float right_angle_deg = 45.0f;
    float collision_radius = 0.065f;
};

struct PidBrain {
    HardwareParams hw;

    // --- Trained parameters (saved in the model file) ---------------------------------------------
    float kp = 1.0f;               // (m/s of turn) per metre of error
    float ki = 0.0f;               // (m/s of turn) per metre*second of accumulated error
    float kd = 0.1f;               // (m/s of turn) per (metre/second) of error change
    float base_speed_frac = 0.65f; // cruise speed / max_linear_speed
    // Experiments (exp_top_speed_mps / exp_wheel_max_mps, 0 = off): forward top speed and wheel ceiling; sat = this
    // tick's requested turn could not be applied in full (a wheel at the ceiling).
    float top_speed = 0.0f, wheel_max = 0.0f;
    bool sat = false;
    // JOB 10 (simulator experiment): the controller chooses its speed, speed = clamp(top - k_slow |err| - k_slow_d |d err/dt|,
    // v_min, top), all three trained by the GA. Used only when the model file has k_slow (speed_law).
    bool fc_new_reading = true;  // JOB 11: false = no new side reading this tick: the D term is not updated
    bool speed_law = false;
    float k_slow = 0.5f, k_slow_d = 0.05f, v_min = 0.06f;

    // --- Fixed shaping, from pid_params.txt (not trained, not saved) --------------------------------
    bool train_base_speed = false;
    float error_clip_m = 0.30f;
    float integral_limit = 0.50f;
    float d_filter_alpha = 0.30f;
    float slowdown_frac = 0.50f;
    int left_idx = 2;   // sensor with the most counter-clockwise (positive) mount angle
    int right_idx = 0;  // sensor with the most clockwise (negative) mount angle

    // --- Runtime state (reset at the start of every episode / after every scan) ---------------------
    float integral = 0.0f;
    float prev_err = 0.0f;
    float d_filtered = 0.0f;
    bool has_prev = false;

    static constexpr float GAIN_MIN = 1e-4f;  // multiplicative mutation can approach 0, never cross it
    static constexpr float GAIN_MAX = 50.0f;

    PidBrain() { InitRandomGains(1337); }

    PidBrain(const SimConfig& cfg, unsigned int seed = 1337) {
        hw.track_width = cfg.track_width_m;
        hw.wheel_radius = cfg.wheel_radius_m;
        hw.max_linear_speed = cfg.max_linear_speed_mps;
        top_speed = cfg.exp_top_speed_mps; wheel_max = cfg.exp_wheel_max_mps;
        hw.left_angle_deg = cfg.sensors[0].angle_deg;
        hw.center_angle_deg = cfg.sensors[1].angle_deg;
        hw.right_angle_deg = cfg.sensors[2].angle_deg;
        hw.collision_radius = cfg.chassis_collision_radius_m;

        base_speed_frac = cfg.pid_base_speed_frac;
        train_base_speed = cfg.pid_train_base_speed;
        error_clip_m = cfg.pid_error_clip_m;
        integral_limit = cfg.pid_integral_limit;
        d_filter_alpha = std::clamp(cfg.pid_d_filter_alpha, 0.0f, 1.0f);
        slowdown_frac = std::clamp(cfg.pid_slowdown_frac, 0.0f, 1.0f);

        // Angles are measured counter-clockwise, so the largest angle looks left.
        left_idx = 0;
        right_idx = 0;
        for (int i = 1; i < 3; ++i) {
            if (cfg.sensors[i].angle_deg > cfg.sensors[left_idx].angle_deg) left_idx = i;
            if (cfg.sensors[i].angle_deg < cfg.sensors[right_idx].angle_deg) right_idx = i;
        }
        InitRandomGains(seed);
    }

    // Log-uniform starting gains: the right gain could be 0.3 or 3, so sample orders of magnitude.
    void InitRandomGains(unsigned int seed) {
        std::mt19937 rng(seed);
        auto log_uniform = [&](float lo, float hi) {
            std::uniform_real_distribution<float> u(std::log(lo), std::log(hi));
            return std::exp(u(rng));
        };
        kp = log_uniform(0.1f, 5.0f);
        ki = log_uniform(0.001f, 0.5f);
        kd = log_uniform(0.001f, 0.5f);
        if (train_base_speed) {
            std::uniform_real_distribution<float> u(0.3f, 0.9f);
            base_speed_frac = u(rng);
        }
    }

    void ResetState() {
        integral = 0.0f;
        prev_err = 0.0f;
        d_filtered = 0.0f;
        has_prev = false;
    }

    // One control tick. dists are the filtered ToF readings in metres (same ones the NN sees).
    void Control(const float dists[3], float dt, float& v_l, float& v_r) {
        const float max_v = wheel_max > 0.0f ? wheel_max : hw.max_linear_speed;
        const float top_v = top_speed > 0.0f ? top_speed : max_v;

        // P: where we are now. Clipped so an opening in one wall (a gap reads 4 m) cannot command
        // a violent turn on its own.
        float err = std::clamp(dists[left_idx] - dists[right_idx], -error_clip_m, error_clip_m);

        // I: accumulated error, with anti-windup so a long transient cannot charge it up forever.
        integral = std::clamp(integral + err * dt, -integral_limit, integral_limit);

        // D: rate of change, low-pass filtered because differentiating noisy readings amplifies noise.
        if (!fc_new_reading && has_prev) {  // JOB 11 fixed clock: hold the D term until a new reading
            float turn_h = std::clamp(kp * err + ki * integral + kd * d_filtered, -max_v, max_v);
            float base_h = base_speed_frac * top_v * (1.0f - slowdown_frac * std::abs(turn_h) / max_v);
            if (speed_law) base_h = std::clamp(top_v - k_slow * std::abs(err) - k_slow_d * std::abs(d_filtered), std::min(v_min, top_v), top_v);
            v_l = std::clamp(base_h - turn_h, -max_v, max_v);
            v_r = std::clamp(base_h + turn_h, -max_v, max_v);
            sat = std::abs((kp * err + ki * integral + kd * d_filtered) - 0.5f * (v_r - v_l)) > 0.002f;
            return;
        }
        float raw_d = has_prev ? (err - prev_err) / dt : 0.0f;
        // d_filter_alpha is defined per 50 ms (the period it was tuned at). For another period the same
        // filter time constant needs alpha_dt = 1 - (1 - alpha)^(dt / 0.05).
        float alpha = 1.0f - std::pow(1.0f - d_filter_alpha, dt / 0.05f);
        d_filtered = alpha * raw_d + (1.0f - alpha) * d_filtered;
        prev_err = err;
        has_prev = true;

        float turn = kp * err + ki * integral + kd * d_filtered;
        turn = std::clamp(turn, -max_v, max_v);

        // Slow down while turning hard (like a driver in a sharp bend), then split into wheels.
        float base = base_speed_frac * top_v * (1.0f - slowdown_frac * std::abs(turn) / max_v);
        if (speed_law) base = std::clamp(top_v - k_slow * std::abs(err) - k_slow_d * std::abs(d_filtered), std::min(v_min, top_v), top_v);
        v_l = std::clamp(base - turn, -max_v, max_v);
        v_r = std::clamp(base + turn, -max_v, max_v);
        // Saturated: more than 2 mm/s of the requested turn was lost to the turn or wheel clamps.
        sat = std::abs((kp * err + ki * integral + kd * d_filtered) - 0.5f * (v_r - v_l)) > 0.002f;
    }

    // Each gain is scaled by exp(N(0, 0.25)), i.e. about +-25%, so small and large gains are
    // explored at the same relative step. At least one parameter always changes.
    void Mutate(std::mt19937& rng) {
        std::normal_distribution<float> step(0.0f, 0.25f);
        std::uniform_real_distribution<float> chance(0.0f, 1.0f);
        std::uniform_int_distribution<int> pick(0, train_base_speed ? 3 : 2);
        int forced = pick(rng);

        float* gains[3] = {&kp, &ki, &kd};
        for (int i = 0; i < 3; ++i) {
            if (i == forced || chance(rng) < 0.5f) {
                *gains[i] = std::clamp(*gains[i] * std::exp(step(rng)), GAIN_MIN, GAIN_MAX);
            }
        }
        if (train_base_speed && (forced == 3 || chance(rng) < 0.5f)) {
            std::normal_distribution<float> speed_step(0.0f, 0.05f);
            base_speed_frac = std::clamp(base_speed_frac + speed_step(rng), 0.2f, 1.0f);
        }
        if (speed_law) {  // JOB 10 (only these models draw more numbers)
            if (chance(rng) < 0.5f) k_slow = std::clamp(std::max(k_slow, 1e-3f) * std::exp(step(rng)), 0.0f, 20.0f);
            if (chance(rng) < 0.5f) k_slow_d = std::clamp(std::max(k_slow_d, 1e-4f) * std::exp(step(rng)), 0.0f, 5.0f);
            if (chance(rng) < 0.5f) { std::normal_distribution<float> vs(0.0f, 0.01f); v_min = std::clamp(v_min + vs(rng), 0.03f, 0.15f); }
        }
    }

    bool Save(const std::string& filename) const {
        std::ofstream f(filename);
        if (!f.is_open()) return false;
        f << "controller pid\n";
        f << "kp " << kp << "\n";
        f << "ki " << ki << "\n";
        f << "kd " << kd << "\n";
        f << "base_speed_frac " << base_speed_frac << "\n";
        f << "error_clip_m " << error_clip_m << "\n";
        if (speed_law) f << "k_slow " << k_slow << "\nk_slow_d " << k_slow_d << "\nv_min " << v_min << "\n";
        f << "track_width " << hw.track_width << "\n";
        return true;
    }

    // base_speed_frac in the file is used only when pid_train_base_speed is on; otherwise the
    // value from pid_params.txt wins, so the cruise speed can be changed without retraining.
    bool Load(const std::string& filename) {
        std::ifstream f(filename);
        if (!f.is_open()) return false;
        std::string key;
        if (!(f >> key) || key != "controller") {
            std::cerr << "Load: " << filename << " is not a PID model (an NN model file, perhaps?).\n";
            return false;
        }
        std::string kind;
        f >> kind;
        if (kind != "pid") {
            std::cerr << "Load: " << filename << " is a '" << kind << "' model; this build expects 'pid'.\n";
            return false;
        }
        bool got_kp = false, got_ki = false, got_kd = false;
        float file_base = base_speed_frac;
        while (f >> key) {
            float v = 0.0f;
            if (!(f >> v)) break;
            if (key == "kp") { kp = v; got_kp = true; }
            else if (key == "ki") { ki = v; got_ki = true; }
            else if (key == "kd") { kd = v; got_kd = true; }
            else if (key == "base_speed_frac") file_base = v;
            else if (key == "track_width") hw.track_width = v;
            else if (key == "error_clip_m") error_clip_m = v;
            else if (key == "k_slow") { k_slow = v; speed_law = true; }
            else if (key == "k_slow_d") { k_slow_d = v; speed_law = true; }
            else if (key == "v_min") { v_min = v; speed_law = true; }  // optional: overrides pid_params.txt (narrow-corridor models)
        }
        if (!got_kp || !got_ki || !got_kd) {
            std::cerr << "Load: " << filename << " is missing kp, ki or kd.\n";
            return false;
        }
        if (train_base_speed) base_speed_frac = file_base;
        return true;
    }
};
