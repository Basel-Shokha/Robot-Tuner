#pragma once
#include <cmath>
#include <string>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

// The simulator's control step (s). Set by LoadConfig from control_period_s (sim_config.txt, generated from
// robot_hardware.txt): the outer controller runs once per ToF period. Every time constant in the sim is in
// seconds, so this can change without changing behaviour (except discretisation).
inline float SIMULATION_DT = 0.05f;

struct SensorMount {
    float x_m{0.0f};
    float y_m{0.0f};
    float angle_deg{0.0f};
};

struct SimConfig {
    // Episode length in control steps at the current SIMULATION_DT.
    int MaxSteps() const;
    // ToF glitch-filter hold limit in control steps.
    int GlitchHoldSteps() const;
    float track_width_m = 0.120f;
    float wheel_radius_m = 0.032f;
    float max_linear_speed_mps = 0.45f;
    float side_angle_min_deg = 20.0f;
    float side_angle_max_deg = 90.0f;
    float side_angle_start_deg = 45.0f;
    // Tracks: train_tracks (train/benchmark, comma-separated) and view_track (visualizer).
    // Resolved in LoadConfig; never empty afterwards.
    std::vector<std::string> train_tracks;
    std::vector<std::string> heldout_tracks;  // benchmark --heldout only; never used for training (tasks.txt C5)
    std::string view_track;

    // Detailed Physical Geometry & Mounts
    float chassis_length_m = 0.10f;
    float chassis_width_m = 0.08f;
    float wheel_offset_from_center_m = 0.03f;
    float wheel_length_m = 0.04f;
    float wheel_width_m = 0.015f;
    float chassis_collision_radius_m = 0.065f;
    // Body outline relative to the robot origin = the drive-axle midpoint (rotation centre).
    // <= 0 means "not given": the legacy split (60% of chassis_length_m ahead, 40% behind) is used.
    float chassis_front_m = -1.0f;
    float chassis_rear_m = -1.0f;
    float caster_x_m = 0.0f;          // drawn only: a passive ball inside the outline never touches a wall
    float caster_diameter_m = 0.0f;
    // Rates. The sim steps at SIMULATION_DT; these document the hardware and are checked against it.
    float tof_period_s = 0.05f;
    float control_period_s = 0.05f;
    SensorMount sensors[3] = {
        {0.0f, 0.0f, -45.0f}, // Sensor 0: Left
        {0.0f, 0.0f,   0.0f}, // Sensor 1: Center
        {0.0f, 0.0f,  45.0f}  // Sensor 2: Right
    };

    // Noise & Physics
    float tof_max_range_m = 4.0f;
    float tof_min_range_m = 0.03f;   // readings are clamped up to this (VL53L1X: ranging starts at 4 cm)
    // --- Actuator / sensor timing (tasks.txt C1, C2, C4). Defaults = legacy behaviour. ------------
    // motor_tau_s: first-order lag of each wheel (closed-loop wheel PID + motor), 0 = instant.
    // motor_max_accel_mps2: wheel acceleration limit, 0 = none.
    // encoder_model: 0 = encoder reports the COMMAND + noise (legacy);
    //                1 = encoder reports the wheel's real speed (lag + per-run wheel scale) + noise.
    //                    It still cannot see slip. The scan turn then also measures its rotation from
    //                    the encoders, exactly as the firmware must.
    // tof_delay_steps: the controller sees the ToF reading taken this many control ticks ago.
    float motor_tau_s = 0.0f;
    float motor_max_accel_mps2 = 0.0f;
    int encoder_model = 0;
    int tof_delay_steps = 0;
    // spawn_at_body_center: 1 = a track's spawn point is where the CENTRE OF THE BODY OUTLINE goes
    // (the axle is placed behind it), 0 = the spawn point is the robot origin (legacy).
    int spawn_at_body_center = 0;
    // --- Inner loop (MotorModel.hpp). wheel_model 0 = "simple" (motor_tau_s lag, legacy),
    //     1 = "pid": the firmware's wheel PID (wheel_pid.h) drives a DC-motor model in sim_substeps
    //     sub-steps, encoders quantised to enc_ticks_per_rev. Hardware values come from sim_config.txt.
    int wheel_model = 0;
    float speed_scale = 1.0f;          // multiplies every wheel target (firmware speed_scale); 1 = as commanded
    int sim_substeps = 10;
    float battery_v = 7.4f;            // pack voltage in the sim (motor speed scales with V / motor_nominal_v)
    float motor_mismatch_std = 0.0f;   // per-run, per-wheel motor gain error (the PID can correct it)
    float enc_ticks_per_rev = 1000.0f;
    float pwm_limit = 120.0f, pwm_max = 255.0f, wheel_pid_period_ms = 50.0f;
    float wheel_kp = 0.4f, wheel_ki = 0.001f, wheel_kd = 0.05f, wheel_ff_static = 35.0f, wheel_ff_gain = 2.0f;
    float motor_k_l = 0.00318f, motor_k_r = 0.00318f, motor_static_l = 35.0f, motor_static_r = 35.0f;
    float motor_tau_l_s = 0.08f, motor_tau_r_s = 0.08f, motor_nominal_v = 7.4f;
    float motor_breakaway_l = 0.0f, motor_breakaway_r = 0.0f;  // PWM to start a wheel from rest
    float tof_noise_std_m = 0.008f;
    float tof_noise_pct_of_range = 0.01f; // fraction of measured distance (0.01 = 1%)
    float tof_dropout_rate = 0.02f;
    float wheel_slip_coefficient = 0.02f;
    float encoder_noise_base = 0.005f;
    float encoder_noise_speed_coeff = 0.02f;
    float tof_glitch_threshold_m = 0.3f;
    // How many consecutive rejected-high readings before the filter gives up and accepts reality.
    // Without this the filter latches forever at a real opening (corridor end, side branch) and
    // reports the old short distance until something comes closer again.
    int tof_glitch_max_hold_steps = 3;
    float tof_glitch_max_hold_s = 0.0f;  // > 0: the hold limit in seconds (overrides the steps above)
    // ToF reading age (s) when the controller uses it: centre of the exposure (budget/2) + read-out. The rays
    // are cast from the robot's pose that long ago (interpolated). Plus uniform jitter per reading.
    float tof_latency_s = 0.0f;
    float tof_latency_jitter_s = 0.0f;
    float tof_timing_budget_s = 0.05f;
    // After the emergency scan the robot drives straight at 0.15 m/s for this long before the controller
    // takes over again (was exactly one 50 ms tick).
    float scan_resume_s = 0.05f;
    // Emergency manoeuvre (firmware/robot_fw/emergency.h): stop, reverse, steer away. error_params.txt keys.
    int emerg_on = 1;
    // Output smoothing (firmware/robot_fw/steer_smooth.h)
    int steer_smooth = 1;
    float steer_tau_s = 0.10f, steer_turn_accel = 0.6f, steer_lin_accel = 0.5f, steer_max_step = 0.01f;
    // Reward (train.cpp): weight of the wheel-command change penalty (was a fixed 5)
    float smooth_weight = 5.0f;
    // Random start pose (Spawn.hpp): wall clearance kept at the spawn, and the smallest |heading offset| as a
    // fraction of the largest feasible one (so the start is never parallel to the walls).
    float spawn_margin_m = 0.010f, spawn_heading_min_frac = 0.33f;
    float emerg_front_m = 0.12f, emerg_side_m = 0.075f, emerg_stop_s = 0.10f, emerg_back_speed = 0.08f;
    float emerg_turn_frac = 0.6f, emerg_back_m = 0.06f, emerg_turn_max_rad = 0.35f, emerg_max_s = 1.5f;
    // Side-wall reflex (firmware/robot_fw/reflex.h), benchmark only.
    int reflex_on = 0;
    float reflex_trigger_m = 0.09f, reflex_halt_s = 0.10f, reflex_turn_deg = 45.0f, reflex_turn_speed = 0.08f;
    int slow_on = 0, slow_keep_turn = 0;
    float slow_m = 0.10f, slow_frac = 0.5f;
    float wall_noise_std_m = 0.010f;

    // --- ToF sensor model selection -----------------------------------------------------------
    // tof_signal_model = 0 uses the cheap nearest-surface-in-cone min(); 1 uses the returned-signal
    // histogram model in Raycast.hpp. Default is the cheap path, so nothing changes unless asked.
    // tof_rays is how many rays span the cone in either model; 3 is the historical value.
    bool tof_signal_model = false;
    int tof_rays = 3;
    // Experiments only (docs/results/speed_sweep): 0 = off (the trained behaviour, bit-exact with the firmware).
    // exp_top_speed_mps: the controller's FORWARD top speed (PID / LineFit cruise, NN both outputs, NN-map v output).
    // exp_wheel_max_mps: the wheel speed ceiling the controllers clamp to (with a stronger motor = a virtual max).
    float exp_top_speed_mps = 0.0f;
    float exp_wheel_max_mps = 0.0f;
    // JOB 11 (comparison only, 0 = off): the controller runs on a fixed clock (control_period_s) and takes the NEWEST
    // reading of each sensor; each sensor ranges on its own (period tof_period_s x (1 + N(0, fc_clock_tol)), random phase),
    // a sample is usable fc_read_s after it completes and describes the pose budget/2 before completion.
    int fixed_clock = 0;
    float fc_clock_tol = 0.01f, fc_read_s = 0.002f;

    // --- ToF field of view ----------------------------------------------------------------------
    // Datasheets give the DIAGONAL FoV of the (square) receiver. Against vertical walls only the
    // horizontal extent matters: tan(h/2) = tan(diag/2) / sqrt(2). VL53L1X, full 16x16 ROI: 27 deg
    // diagonal -> 19.27 deg horizontal (half-angle 9.64 deg). Set tof_fov_horizontal_deg to override
    // the derivation (non-square ROI, other sensor). LoadConfig fills tof_cone_half_angle_rad.
    float tof_fov_diagonal_deg = 27.0f;
    float tof_fov_horizontal_deg = 0.0f;  // 0 = derive from the diagonal
    float tof_cone_half_angle_rad = 0.16816f;

    // --- Signal model coefficients (ignored when tof_signal_model is 0) ------------------------
    // Provenance of each value is marked DATASHEET (published, see Raycast.hpp), DERIVED
    // (calculated from published values) or ASSUMED (not published; chosen and named so it can be
    // argued with). All are runtime-configurable from error_params.txt.
    float tof_signal_gain_kcps_m2 = 15080.0f;  // DERIVED: 1024 * 3.60^2 / 0.88, from Table 6
    float tof_signal_threshold_kcps = 1024.0f; // DATASHEET/ULD: VL53L1X_SetSignalThreshold default
    float tof_wall_reflectance = 0.54f;        // DATASHEET grade: grey 54% (N8.25), a painted wall
    float tof_cos_power = 2.0f;                // ASSUMED: textbook Lambertian is 1.0; 2.0 allows for
                                               //   the sharper grazing dropout ToF modules show
    float tof_offaxis_sigma_frac = 0.60f;      // ASSUMED: Gaussian sigma as a fraction of the cone
                                               //   half-angle; edge ray carries ~25% of centre weight
    float tof_peak_merge_m = 0.05f;            // ASSUMED: histogram bin width for merging returns
    float tof_weak_signal_sigma_cap = 4.0f;    // ASSUMED: cap on the 1/sqrt(signal) noise growth
    float approach_deadband_mps = 0.25f;  // m/s; noise floor, roughly tof_noise_std_m*1.41/SIMULATION_DT
    float centering_deadband_lo = 0.06f;  // meters; below this, reward squared
    float centering_deadband_hi = 0.07f;  // meters; above this, penalize squared
    float centering_weight = 3.0f;        // weight applied to centering_score
    float finish_bonus = 500.0f;          // reward for reaching the finish line (scaled by time remaining)
    float speed_bonus = 0.0f;             // finished runs: + speed_bonus * average speed / top speed (0 = off)
    // Per-run systematic hardware error, drawn once per episode and constant for the whole run.
    // This is the sim-to-real gap the per-tick noise above does not model: real hardware fails with
    // biases, not with zero-mean jitter. 0 disables, which is the default so existing runs are
    // unchanged. Suggested starting values: 2.0 / 0.02 / 0.05.
    float sensor_angle_bias_std_deg = 0.0f;  // ToF glued on a few degrees off, per sensor
    float wheel_scale_bias_std = 0.0f;       // left/right wheel+motor mismatch, as a fraction
    float max_speed_bias_std = 0.0f;         // pack-to-pack battery variation, as a fraction
    // --- Straight-track reward (Straight_Wall_Project) ------------------------------------------
    // Every metre driven along the track axis earns PROGRESS_WEIGHT x (1 + parallel_weight * p +
    // center_weight * c), where p = exp(-(heading_err / heading_sigma_deg)^2) and
    // c = exp(-(centre_err / (center_sigma_frac * free half width))^2), both measured against
    // StraightGuide's robust axis/centre so wall errors are ignored. Large weights make a metre
    // driven parallel and centred worth many times a sloppy one.
    float parallel_weight = 4.0f;
    float center_weight = 6.0f;
    float heading_sigma_deg = 12.0f;
    float center_sigma_frac = 0.30f;
    float crash_penalty = 200.0f;
    // Wobble: every reversal of turning direction (left->right or right->left, both above
    // wobble_omega_deadband rad/s) costs wobble_weight * min(|omega|, 3).
    float wobble_weight = 60.0f;
    float wobble_omega_deadband = 0.4f;
    // Per-episode spawn randomization (episode 0 always uses the track file's spawn exactly).
    // Heading = track axis + uniform(-spawn_heading_random_deg, +spawn_heading_random_deg);
    // lateral offset = uniform(-1, 1) * spawn_lateral_random_frac * (half width - turning radius).
    float spawn_heading_random_deg = 0.0f;
    float spawn_lateral_random_frac = 0.0f;
    float max_episode_s = 0.0f;           // > 0: episode length in seconds (overrides max_episode_steps)
    int max_episode_steps = 800;          // episode length cap for training and benchmarking.
                                          // Seconds = max_episode_steps * SIMULATION_DT (800 = 40 s).
                                          // A track the robot cannot finish inside this always reads
                                          // out-of-steps: grand_tour needs ~3400.

    // --- PID controller (pid_params.txt) -----------------------------------------------------------
    // These shape the controller but are NOT trained; the trained values (Kp, Ki, Kd and, optionally,
    // the cruise speed) live in the model file (best_pid_model.txt). See PidController.hpp.
    float pid_base_speed_frac = 0.65f;   // cruise speed as a fraction of max_linear_speed_mps
    bool  pid_train_base_speed = false;  // 1 = let training also choose the cruise speed (4th parameter)
    float pid_error_clip_m = 0.30f;      // |left - right| is clipped to this before the PID sees it
    float pid_integral_limit = 0.50f;    // anti-windup: |integral of error| never exceeds this (m*s)
    float pid_d_filter_alpha = 0.30f;    // low-pass on the D term: 1 = raw derivative, smaller = smoother
    float pid_slowdown_frac = 0.50f;     // cruise speed drops by up to this fraction when the turn is maximal
};

// Splits "a, b,c" into {"a","b","c"} (whitespace trimmed, empty entries dropped).
inline std::vector<std::string> SplitTrackList(const std::string& text) {
    std::vector<std::string> out;
    std::stringstream ss(text);
    std::string item;
    while (std::getline(ss, item, ',')) {
        size_t b = item.find_first_not_of(" \t\r");
        size_t e = item.find_last_not_of(" \t\r");
        if (b != std::string::npos) out.push_back(item.substr(b, e - b + 1));
    }
    return out;
}

// Display/file label for a set of tracks: the name for one track, "multi_<N>" otherwise.
inline std::string TrackSetLabel(const std::vector<std::string>& tracks) {
    return tracks.size() == 1 ? tracks[0] : "multi_" + std::to_string(tracks.size());
}

inline SimConfig LoadConfig(const std::string& path) {
    SimConfig config;
    std::string legacy_track_name;  // old single "track_name" key: sets both lists unless they are given explicitly
    bool has_train_key = false, has_view_key = false;
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cout << "Warning: " << path << " not found, using defaults.\n";
    } else {
        std::string line;
        bool first_line = true;
        while (std::getline(file, line)) {
            if (first_line && line.compare(0, 3, "\xEF\xBB\xBF") == 0) line.erase(0, 3);
            first_line = false;
            if (line.empty() || line[0] == '#') continue;
            std::stringstream ss(line);
            std::string key;
            ss >> key;
            if (key == "track_width_m") ss >> config.track_width_m;
            else if (key == "wheel_radius_m") ss >> config.wheel_radius_m;
            else if (key == "max_linear_speed_mps") ss >> config.max_linear_speed_mps;
            else if (key == "side_angle_min_deg") ss >> config.side_angle_min_deg;
            else if (key == "side_angle_max_deg") ss >> config.side_angle_max_deg;
            else if (key == "side_angle_start_deg") ss >> config.side_angle_start_deg;
            else if (key == "track_name") ss >> legacy_track_name;
            else if (key == "train_tracks") {
                std::string rest;
                std::getline(ss, rest);
                config.train_tracks = SplitTrackList(rest);
                has_train_key = true;
            }
            else if (key == "heldout_tracks") {
                std::string rest;
                std::getline(ss, rest);
                config.heldout_tracks = SplitTrackList(rest);
            }
            else if (key == "view_track") { ss >> config.view_track; has_view_key = true; }
            else if (key == "chassis_length_m") ss >> config.chassis_length_m;
            else if (key == "chassis_width_m") ss >> config.chassis_width_m;
            else if (key == "wheel_offset_from_center_m") ss >> config.wheel_offset_from_center_m;
            else if (key == "wheel_length_m") ss >> config.wheel_length_m;
            else if (key == "wheel_width_m") ss >> config.wheel_width_m;
            else if (key == "chassis_collision_radius_m") ss >> config.chassis_collision_radius_m;
            else if (key == "chassis_front_m") ss >> config.chassis_front_m;
            else if (key == "chassis_rear_m") ss >> config.chassis_rear_m;
            else if (key == "caster_x_m") ss >> config.caster_x_m;
            else if (key == "caster_diameter_m") ss >> config.caster_diameter_m;
            else if (key == "tof_period_s") ss >> config.tof_period_s;
            else if (key == "control_period_s") ss >> config.control_period_s;
            else if (key == "tof_timing_budget_s") ss >> config.tof_timing_budget_s;
            else if (key == "enc_ticks_per_rev") ss >> config.enc_ticks_per_rev;
            else if (key == "pwm_limit") ss >> config.pwm_limit;
            else if (key == "pwm_max") ss >> config.pwm_max;
            else if (key == "wheel_pid_period_ms") ss >> config.wheel_pid_period_ms;
            else if (key == "wheel_kp") ss >> config.wheel_kp;
            else if (key == "wheel_ki") ss >> config.wheel_ki;
            else if (key == "wheel_kd") ss >> config.wheel_kd;
            else if (key == "wheel_ff_static") ss >> config.wheel_ff_static;
            else if (key == "wheel_ff_gain") ss >> config.wheel_ff_gain;
            else if (key == "motor_k_l") ss >> config.motor_k_l;
            else if (key == "motor_k_r") ss >> config.motor_k_r;
            else if (key == "motor_static_l") ss >> config.motor_static_l;
            else if (key == "motor_static_r") ss >> config.motor_static_r;
            else if (key == "motor_tau_l_s") ss >> config.motor_tau_l_s;
            else if (key == "motor_tau_r_s") ss >> config.motor_tau_r_s;
            else if (key == "motor_nominal_v") ss >> config.motor_nominal_v;
            else if (key == "motor_breakaway_l") ss >> config.motor_breakaway_l;
            else if (key == "motor_breakaway_r") ss >> config.motor_breakaway_r;
            // ToF hardware keys may live in sim_config.txt (generated from robot_hardware.txt).
            else if (key == "tof_max_range_m") ss >> config.tof_max_range_m;
            else if (key == "tof_min_range_m") ss >> config.tof_min_range_m;
            else if (key == "tof_fov_diagonal_deg") ss >> config.tof_fov_diagonal_deg;
            else if (key == "sensor_0_pose") ss >> config.sensors[0].x_m >> config.sensors[0].y_m >> config.sensors[0].angle_deg;
            else if (key == "sensor_1_pose") ss >> config.sensors[1].x_m >> config.sensors[1].y_m >> config.sensors[1].angle_deg;
            else if (key == "sensor_2_pose") ss >> config.sensors[2].x_m >> config.sensors[2].y_m >> config.sensors[2].angle_deg;
        }
    }

    if (!has_train_key && !legacy_track_name.empty()) config.train_tracks = {legacy_track_name};
    if (!has_view_key && !legacy_track_name.empty()) config.view_track = legacy_track_name;
    if (config.train_tracks.empty()) config.train_tracks = {config.view_track.empty() ? "benchmark" : config.view_track};
    if (config.view_track.empty()) config.view_track = config.train_tracks[0];

    std::ifstream err_file("error_params.txt");
    if (!err_file.is_open()) {
        std::cout << "Warning: error_params.txt not found, using defaults.\n";
    } else {
        std::string line;
        bool first_line = true;
        while (std::getline(err_file, line)) {
            if (first_line && line.compare(0, 3, "\xEF\xBB\xBF") == 0) line.erase(0, 3);
            first_line = false;
            if (line.empty() || line[0] == '#') continue;
            std::stringstream ss(line);
            std::string key;
            ss >> key;
            if (key == "tof_max_range_m") ss >> config.tof_max_range_m;
            else if (key == "tof_noise_std_m") ss >> config.tof_noise_std_m;
            else if (key == "tof_min_range_m") ss >> config.tof_min_range_m;
            else if (key == "motor_tau_s") ss >> config.motor_tau_s;
            else if (key == "motor_max_accel_mps2") ss >> config.motor_max_accel_mps2;
            else if (key == "encoder_model") ss >> config.encoder_model;
            else if (key == "tof_delay_steps") ss >> config.tof_delay_steps;
            else if (key == "spawn_at_body_center") ss >> config.spawn_at_body_center;
            else if (key == "wheel_model") { std::string m; ss >> m; config.wheel_model = (m == "pid" || m == "1") ? 1 : 0; }
            else if (key == "sim_substeps") ss >> config.sim_substeps;
            else if (key == "speed_scale") ss >> config.speed_scale;
            else if (key == "battery_v") ss >> config.battery_v;
            else if (key == "motor_mismatch_std") ss >> config.motor_mismatch_std;
            else if (key == "tof_noise_pct_of_range") ss >> config.tof_noise_pct_of_range;
            else if (key == "tof_dropout_rate") ss >> config.tof_dropout_rate;
            else if (key == "wheel_slip_coefficient") ss >> config.wheel_slip_coefficient;
            else if (key == "encoder_noise_base") ss >> config.encoder_noise_base;
            else if (key == "encoder_noise_speed_coeff") ss >> config.encoder_noise_speed_coeff;
            else if (key == "tof_glitch_threshold_m") ss >> config.tof_glitch_threshold_m;
            else if (key == "tof_glitch_max_hold_steps") ss >> config.tof_glitch_max_hold_steps;
            else if (key == "tof_glitch_max_hold_s") ss >> config.tof_glitch_max_hold_s;
            else if (key == "tof_latency_s") ss >> config.tof_latency_s;
            else if (key == "tof_latency_jitter_s") ss >> config.tof_latency_jitter_s;
            else if (key == "scan_resume_s") ss >> config.scan_resume_s;
            else if (key == "emerg_on") ss >> config.emerg_on;
            else if (key == "steer_smooth") ss >> config.steer_smooth;
            else if (key == "steer_tau_s") ss >> config.steer_tau_s;
            else if (key == "steer_turn_accel") ss >> config.steer_turn_accel;
            else if (key == "steer_lin_accel") ss >> config.steer_lin_accel;
            else if (key == "steer_max_step") ss >> config.steer_max_step;
            else if (key == "smooth_weight") ss >> config.smooth_weight;
            else if (key == "spawn_margin_m") ss >> config.spawn_margin_m;
            else if (key == "spawn_heading_min_frac") ss >> config.spawn_heading_min_frac;
            else if (key == "emerg_front_m") ss >> config.emerg_front_m;
            else if (key == "emerg_side_m") ss >> config.emerg_side_m;
            else if (key == "emerg_stop_s") ss >> config.emerg_stop_s;
            else if (key == "emerg_back_speed") ss >> config.emerg_back_speed;
            else if (key == "emerg_turn_frac") ss >> config.emerg_turn_frac;
            else if (key == "emerg_back_m") ss >> config.emerg_back_m;
            else if (key == "emerg_turn_max_rad") ss >> config.emerg_turn_max_rad;
            else if (key == "emerg_max_s") ss >> config.emerg_max_s;
            else if (key == "reflex_on") ss >> config.reflex_on;
            else if (key == "reflex_trigger_m") ss >> config.reflex_trigger_m;
            else if (key == "reflex_halt_s") ss >> config.reflex_halt_s;
            else if (key == "reflex_turn_deg") ss >> config.reflex_turn_deg;
            else if (key == "reflex_turn_speed") ss >> config.reflex_turn_speed;
            else if (key == "slow_on") ss >> config.slow_on;
            else if (key == "slow_m") ss >> config.slow_m;
            else if (key == "slow_frac") ss >> config.slow_frac;
            else if (key == "slow_keep_turn") ss >> config.slow_keep_turn;
            else if (key == "max_episode_s") ss >> config.max_episode_s;
            else if (key == "wall_noise_std_m") ss >> config.wall_noise_std_m;
            else if (key == "tof_signal_model") { int v = 0; ss >> v; config.tof_signal_model = (v != 0); }
            else if (key == "tof_rays") ss >> config.tof_rays;
            else if (key == "exp_top_speed_mps") ss >> config.exp_top_speed_mps;
            else if (key == "exp_wheel_max_mps") ss >> config.exp_wheel_max_mps;
            else if (key == "fixed_clock") ss >> config.fixed_clock;
            else if (key == "fc_clock_tol") ss >> config.fc_clock_tol;
            else if (key == "fc_read_s") ss >> config.fc_read_s;
            else if (key == "tof_fov_diagonal_deg") ss >> config.tof_fov_diagonal_deg;
            else if (key == "tof_fov_horizontal_deg") ss >> config.tof_fov_horizontal_deg;
            else if (key == "tof_signal_gain_kcps_m2") ss >> config.tof_signal_gain_kcps_m2;
            else if (key == "tof_signal_threshold_kcps") ss >> config.tof_signal_threshold_kcps;
            else if (key == "tof_wall_reflectance") ss >> config.tof_wall_reflectance;
            else if (key == "tof_cos_power") ss >> config.tof_cos_power;
            else if (key == "tof_offaxis_sigma_frac") ss >> config.tof_offaxis_sigma_frac;
            else if (key == "tof_peak_merge_m") ss >> config.tof_peak_merge_m;
            else if (key == "tof_weak_signal_sigma_cap") ss >> config.tof_weak_signal_sigma_cap;
            else if (key == "approach_deadband_mps") ss >> config.approach_deadband_mps;
            else if (key == "centering_deadband_lo") ss >> config.centering_deadband_lo;
            else if (key == "centering_deadband_hi") ss >> config.centering_deadband_hi;
            else if (key == "centering_weight") ss >> config.centering_weight;
            else if (key == "finish_bonus") ss >> config.finish_bonus;
            else if (key == "speed_bonus") ss >> config.speed_bonus;
            else if (key == "max_episode_steps") ss >> config.max_episode_steps;
            else if (key == "parallel_weight") ss >> config.parallel_weight;
            else if (key == "center_weight") ss >> config.center_weight;
            else if (key == "heading_sigma_deg") ss >> config.heading_sigma_deg;
            else if (key == "center_sigma_frac") ss >> config.center_sigma_frac;
            else if (key == "crash_penalty") ss >> config.crash_penalty;
            else if (key == "wobble_weight") ss >> config.wobble_weight;
            else if (key == "wobble_omega_deadband") ss >> config.wobble_omega_deadband;
            else if (key == "spawn_heading_random_deg") ss >> config.spawn_heading_random_deg;
            else if (key == "spawn_lateral_random_frac") ss >> config.spawn_lateral_random_frac;
            else if (key == "sensor_angle_bias_std_deg") ss >> config.sensor_angle_bias_std_deg;
            else if (key == "wheel_scale_bias_std") ss >> config.wheel_scale_bias_std;
            else if (key == "max_speed_bias_std") ss >> config.max_speed_bias_std;
        }
    }

    std::ifstream pid_file("pid_params.txt");
    if (!pid_file.is_open()) {
        std::cout << "Warning: pid_params.txt not found, using PID defaults.\n";
    } else {
        std::string line;
        bool first_line = true;
        while (std::getline(pid_file, line)) {
            if (first_line && line.compare(0, 3, "\xEF\xBB\xBF") == 0) line.erase(0, 3);
            first_line = false;
            if (line.empty() || line[0] == '#') continue;
            std::stringstream ss(line);
            std::string key;
            ss >> key;
            if (key == "pid_base_speed_frac") ss >> config.pid_base_speed_frac;
            else if (key == "pid_train_base_speed") { int v = 0; ss >> v; config.pid_train_base_speed = (v != 0); }
            else if (key == "pid_error_clip_m") ss >> config.pid_error_clip_m;
            else if (key == "pid_integral_limit") ss >> config.pid_integral_limit;
            else if (key == "pid_d_filter_alpha") ss >> config.pid_d_filter_alpha;
            else if (key == "pid_slowdown_frac") ss >> config.pid_slowdown_frac;
        }
    }
    // The sim steps at the control period (the ToF period on the robot).
    if (config.control_period_s > 0.0005f) SIMULATION_DT = config.control_period_s;
    {
        const float d2r = 3.14159265f / 180.0f;
        config.tof_cone_half_angle_rad = config.tof_fov_horizontal_deg > 0.0f
            ? 0.5f * config.tof_fov_horizontal_deg * d2r
            : std::atan(std::tan(0.5f * config.tof_fov_diagonal_deg * d2r) / std::sqrt(2.0f));
    }
    return config;
}
inline int SimConfig::MaxSteps() const {
    return max_episode_s > 0.0f ? (int)std::lround(max_episode_s / SIMULATION_DT) : max_episode_steps;
}
inline int SimConfig::GlitchHoldSteps() const {
    return tof_glitch_max_hold_s > 0.0f ? (int)std::lround(tof_glitch_max_hold_s / SIMULATION_DT) : tof_glitch_max_hold_steps;
}
