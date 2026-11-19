// equiv_upstream.cpp - the Robot Tuner engine must reproduce the UNMODIFIED upstream simulator bit for bit.
//
// Builds the upstream PID simulator headers (vendor/simulation-to-real-life/PID: Robot.hpp, PidController.hpp,
// MotorModel.hpp, Track.hpp, Spawn.hpp + the firmware headers) next to sim/rt_engine.hpp and runs both side by
// side on upstream tracks, for several configurations. Every tick the pose, wheel commands, encoder speeds and
// all sensor readings must have identical bits. Also checks the portable distributions against libstdc++,
// PerturbTrack and RandomSpawn.
// Build: compiled with g++ (libstdc++) and RT_PORTABLE_MATH=0, i.e. exactly the upstream build's arithmetic.
#define RT_PORTABLE_MATH 0
#include "../../vendor/simulation-to-real-life/PID/Robot.hpp"
#include "../../vendor/simulation-to-real-life/PID/Track.hpp"
#include "../../vendor/simulation-to-real-life/PID/Spawn.hpp"
#include "../../sim/rt_engine.hpp"
#include <cstdio>
#include <cstring>
#include <string>

static int g_fail = 0;
static bool Same(float a, float b) { return std::memcmp(&a, &b, sizeof a) == 0; }
#define CHECK(cond, ...) do { if (!(cond)) { ++g_fail; std::printf("FAIL: " __VA_ARGS__); std::printf("\n"); } } while (0)

static const char* VENDOR = "vendor/simulation-to-real-life/";

// Upstream SimConfig as error_params.txt / sim_config.txt of the real robot (Nov 2026), then per-scenario tweaks.
static SimConfig RealRobot() {
    SimConfig c;
    c.track_width_m = 0.1249f; c.wheel_radius_m = 0.0225f; c.max_linear_speed_mps = 0.156f;
    c.chassis_front_m = 0.14f; c.chassis_rear_m = 0.04f; c.chassis_length_m = 0.18f; c.chassis_width_m = 0.14f;
    c.wheel_offset_from_center_m = 0.0f; c.wheel_length_m = 0.045f; c.wheel_width_m = 0.02f;
    c.sensors[0] = {0.10076f, -0.01024f, -45.0f}; c.sensors[1] = {0.105f, 0.0f, 0.0f}; c.sensors[2] = {0.09864f, 0.00812f, 45.0f};
    c.enc_ticks_per_rev = 8291.5f; c.pwm_limit = 255; c.wheel_pid_period_ms = 10;
    c.motor_k_l = 0.000613f; c.motor_k_r = 0.000618f; c.motor_static_l = 3.2f; c.motor_static_r = 3.8f;
    c.motor_breakaway_l = 20; c.motor_breakaway_r = 20; c.motor_tau_l_s = 0.0362f; c.motor_tau_r_s = 0.04f;
    c.motor_nominal_v = 8.05f; c.pwm_max = 255; c.wheel_kp = 2; c.wheel_ki = 0.001f; c.wheel_kd = 5; c.wheel_ff_gain = 2.785f; c.wheel_ff_static = 4;
    c.tof_max_range_m = 1.3f; c.tof_min_range_m = 0.04f; c.tof_fov_diagonal_deg = 27;
    c.tof_period_s = 0.025f; c.tof_timing_budget_s = 0.02f; c.control_period_s = 0.025f;
    c.tof_noise_std_m = 0.004f; c.tof_dropout_rate = 0.02f; c.tof_glitch_threshold_m = 0.12f; c.tof_glitch_max_hold_s = 2.0f;
    c.encoder_noise_base = 0.005f; c.encoder_noise_speed_coeff = 0.02f; c.wheel_slip_coefficient = 0.02f; c.wall_noise_std_m = 0.01f;
    c.max_episode_s = 240; c.sensor_angle_bias_std_deg = 2.0f; c.wheel_scale_bias_std = 0.02f; c.max_speed_bias_std = 0.05f;
    c.encoder_model = 1; c.motor_tau_s = 0.10f; c.tof_latency_s = 0.0132f; c.tof_latency_jitter_s = 0.018f;
    c.wheel_model = 1; c.sim_substeps = 10; c.battery_v = 8.05f; c.motor_mismatch_std = 0.02f;
    c.spawn_margin_m = 0.02f; c.spawn_heading_min_frac = 0.33f; c.emerg_on = 0; c.emerg_side_m = 0.085f;
    c.steer_smooth = 1; c.steer_tau_s = 0.10f; c.steer_turn_accel = 1.5f; c.steer_lin_accel = 0.5f; c.steer_max_step = 0.0105f;
    c.spawn_heading_random_deg = 30.0f; c.spawn_lateral_random_frac = 0.5f;
    c.pid_base_speed_frac = 1.0f; c.pid_error_clip_m = 0.08f;
    return c;
}

static void Finish(SimConfig& c) {  // what LoadConfig does at the end
    SIMULATION_DT = c.control_period_s;
    const float d2r = 3.14159265f / 180.0f;
    c.tof_cone_half_angle_rad = c.tof_fov_horizontal_deg > 0.0f ? 0.5f * c.tof_fov_horizontal_deg * d2r
        : std::atan(std::tan(0.5f * c.tof_fov_diagonal_deg * d2r) / std::sqrt(2.0f));
}

// The same configuration in the Robot Tuner engine.
static rt::Config ToRt(const SimConfig& s, float kp, float ki, float kd) {
    rt::Config c;
    c.dt = SIMULATION_DT;
    c.track_width_m = s.track_width_m; c.wheel_radius_m = s.wheel_radius_m; c.max_linear_speed_mps = s.max_linear_speed_mps;
    c.chassis_length_m = s.chassis_length_m; c.chassis_width_m = s.chassis_width_m; c.chassis_front_m = s.chassis_front_m; c.chassis_rear_m = s.chassis_rear_m;
    c.wheel_offset_from_center_m = s.wheel_offset_from_center_m; c.wheel_length_m = s.wheel_length_m; c.wheel_width_m = s.wheel_width_m;
    c.n_sensors = 3;
    for (int i = 0; i < 3; ++i) {
        rt::Sensor& m = c.sensors[i];
        m.x_m = s.sensors[i].x_m; m.y_m = s.sensors[i].y_m; m.angle_deg = s.sensors[i].angle_deg;
        m.half_angle_rad = s.tof_cone_half_angle_rad; m.rays = s.tof_rays; m.max_range_m = s.tof_max_range_m; m.min_range_m = s.tof_min_range_m;
        m.noise_std_m = s.tof_noise_std_m; m.noise_pct = s.tof_noise_pct_of_range; m.dropout = s.tof_dropout_rate;
    }
    c.tof_glitch_threshold_m = s.tof_glitch_threshold_m; c.tof_glitch_max_hold_steps = s.tof_glitch_max_hold_steps; c.tof_glitch_max_hold_s = s.tof_glitch_max_hold_s;
    c.tof_latency_s = s.tof_latency_s; c.tof_latency_jitter_s = s.tof_latency_jitter_s; c.tof_delay_steps = s.tof_delay_steps;
    c.motor_tau_s = s.motor_tau_s; c.motor_max_accel_mps2 = s.motor_max_accel_mps2; c.encoder_model = s.encoder_model;
    c.wheel_slip_coefficient = s.wheel_slip_coefficient; c.encoder_noise_base = s.encoder_noise_base; c.encoder_noise_speed_coeff = s.encoder_noise_speed_coeff;
    c.wheel_model = s.wheel_model; c.speed_scale = s.speed_scale; c.sim_substeps = s.sim_substeps; c.battery_v = s.battery_v;
    c.motor_mismatch_std = s.motor_mismatch_std; c.enc_ticks_per_rev = s.enc_ticks_per_rev; c.pwm_limit = s.pwm_limit; c.pwm_max = s.pwm_max;
    c.wheel_pid_period_ms = s.wheel_pid_period_ms; c.wheel_kp = s.wheel_kp; c.wheel_ki = s.wheel_ki; c.wheel_kd = s.wheel_kd;
    c.wheel_ff_static = s.wheel_ff_static; c.wheel_ff_gain = s.wheel_ff_gain; c.motor_k_l = s.motor_k_l; c.motor_k_r = s.motor_k_r;
    c.motor_static_l = s.motor_static_l; c.motor_static_r = s.motor_static_r; c.motor_tau_l_s = s.motor_tau_l_s; c.motor_tau_r_s = s.motor_tau_r_s;
    c.motor_nominal_v = s.motor_nominal_v; c.motor_breakaway_l = s.motor_breakaway_l; c.motor_breakaway_r = s.motor_breakaway_r;
    c.sensor_angle_bias_std_deg = s.sensor_angle_bias_std_deg; c.wheel_scale_bias_std = s.wheel_scale_bias_std; c.max_speed_bias_std = s.max_speed_bias_std;
    c.emerg_on = s.emerg_on; c.emerg_front_m = s.emerg_front_m; c.emerg_side_m = s.emerg_side_m; c.emerg_stop_s = s.emerg_stop_s;
    c.emerg_back_speed = s.emerg_back_speed; c.emerg_turn_frac = s.emerg_turn_frac; c.emerg_back_m = s.emerg_back_m;
    c.emerg_turn_max_rad = s.emerg_turn_max_rad; c.emerg_max_s = s.emerg_max_s;
    c.steer_smooth = s.steer_smooth; c.steer_tau_s = s.steer_tau_s; c.steer_turn_accel = s.steer_turn_accel;
    c.steer_lin_accel = s.steer_lin_accel; c.steer_max_step = s.steer_max_step;
    c.kp = kp; c.ki = ki; c.kd = kd; c.base_speed_frac = s.pid_base_speed_frac;
    c.pid_error_clip_m = s.pid_error_clip_m; c.pid_integral_limit = s.pid_integral_limit; c.pid_d_filter_alpha = s.pid_d_filter_alpha;
    c.pid_slowdown_frac = s.pid_slowdown_frac;
    c.wall_noise_std_m = s.wall_noise_std_m; c.spawn_margin_m = s.spawn_margin_m; c.spawn_heading_min_frac = s.spawn_heading_min_frac;
    c.spawn_heading_random_deg = s.spawn_heading_random_deg; c.spawn_lateral_random_frac = s.spawn_lateral_random_frac;
    return c;
}

static std::vector<rt::Wall> ToRtWalls(const std::vector<Wall>& w) {
    std::vector<rt::Wall> o;
    for (const auto& x : w) o.push_back({{x.p1.x, x.p1.y}, {x.p2.x, x.p2.y}});
    return o;
}

// Run upstream Robot and rt::Sim side by side; returns ticks compared.
static long RunPair(const char* name, SimConfig sc, const std::string& track, unsigned seed, int max_ticks, bool perturb, bool random_spawn, bool cull) {
    Finish(sc);
    LoadedTrack t;
    t.walls = LoadTrackFromFile(std::string(VENDOR) + "tracks/" + track + ".track", t.spawn, t.spawn_heading, &t.finish);
    std::vector<Vec2> cp; std::vector<float> hw;
    if (LoadCenterline(std::string(VENDOR) + "tracks/" + track + ".track", cp, hw)) t.guide.BuildPolyline(cp, hw);
    else t.guide.Build(t.walls, t.spawn, t.finish.has_finish, (t.finish.segment.p1 + t.finish.segment.p2) * 0.5f);

    std::mt19937 track_rng_u(seed ^ 0x55u), track_rng_r(seed ^ 0x55u);
    std::vector<Wall> walls_u = perturb ? PerturbTrack(t.walls, sc.wall_noise_std_m, track_rng_u) : t.walls;
    std::vector<rt::Wall> walls_r = perturb ? rt::PerturbTrack(ToRtWalls(t.walls), sc.wall_noise_std_m, track_rng_r) : ToRtWalls(t.walls);
    CHECK(walls_u.size() == walls_r.size(), "%s: wall count", name);
    for (size_t i = 0; i < walls_u.size(); ++i)
        if (!Same(walls_u[i].p1.x, walls_r[i].p1.x) || !Same(walls_u[i].p1.y, walls_r[i].p1.y) || !Same(walls_u[i].p2.x, walls_r[i].p2.x) || !Same(walls_u[i].p2.y, walls_r[i].p2.y)) {
            CHECK(false, "%s: PerturbTrack wall %zu differs", name, i); break;
        }

    const float kp = 0.586855f, ki = 0.0158843f, kd = 0.384207f;
    rt::Sim r;
    r.cfg = ToRt(sc, kp, ki, kd);
    r.walls = walls_r;
    r.cull = cull;

    Vec2 sp_u = t.spawn; float sh_u = t.spawn_heading;
    rt::V2 sp_r{t.spawn.x, t.spawn.y}; float sh_r = t.spawn_heading;
    if (random_spawn) {
        RandomSpawn(t, sc, walls_u, track_rng_u, sp_u, sh_u);
        rt::Guide g;
        std::vector<rt::V2> p2; for (auto& p : cp) p2.push_back({p.x, p.y});
        g.Build(p2, hw);
        r.Reset(sp_r, sh_r);
        rt::RandomSpawn(r, g, sp_r, sh_r, track_rng_r, sp_r, sh_r);
        CHECK(Same(sp_u.x, sp_r.x) && Same(sp_u.y, sp_r.y) && Same(sh_u, sh_r), "%s: RandomSpawn differs (%g %g %g vs %g %g %g)",
              name, sp_u.x, sp_u.y, sh_u, sp_r.x, sp_r.y, sh_r);
    }

    Robot u;
    u.brain = PidBrain(sc);
    u.brain.kp = kp; u.brain.ki = ki; u.brain.kd = kd;
    u.Reset(sp_u, sh_u, sc.tof_max_range_m);
    std::mt19937 rng_u(seed);
    u.SampleHardwareBias(sc, rng_u);

    r.Reset(sp_r, sh_r);
    rt::Rng rng_r(seed);
    r.SampleHardwareBias(rng_r);

    const Wall* fin_u = t.finish.has_finish ? &t.finish.segment : nullptr;
    rt::Wall fin_r{{t.finish.segment.p1.x, t.finish.segment.p1.y}, {t.finish.segment.p2.x, t.finish.segment.p2.y}};
    u.Step(walls_u, sc, rng_u, fin_u);  // warm-up compare below happens after each step
    r.Step(rng_r, t.finish.has_finish ? &fin_r : nullptr);
    long k = 1;
    for (; k < max_ticks; ++k) {
        bool ok = Same(u.pos.x, r.pos.x) && Same(u.pos.y, r.pos.y) && Same(u.theta, r.theta) && Same(u.v_l, r.v_l) && Same(u.v_r, r.v_r)
               && Same(u.enc_l, r.enc_l) && Same(u.enc_r, r.enc_r) && u.alive == r.alive && u.finished == r.finished;
        for (int i = 0; i < 3; ++i) ok = ok && Same(u.dists[i], r.dists[i]) && Same(u.raw_dists[i], r.raw[i]);
        if (!ok) {
            CHECK(false, "%s: tick %ld differs: upstream x %.9g y %.9g th %.9g vl %.9g vr %.9g d %.6g %.6g %.6g | rt x %.9g y %.9g th %.9g vl %.9g vr %.9g d %.6g %.6g %.6g",
                  name, k, u.pos.x, u.pos.y, u.theta, u.v_l, u.v_r, u.dists[0], u.dists[1], u.dists[2],
                  r.pos.x, r.pos.y, r.theta, r.v_l, r.v_r, r.dists[0], r.dists[1], r.dists[2]);
            return k;
        }
        if (!u.alive || u.finished) break;
        u.Step(walls_u, sc, rng_u, fin_u);
        r.Step(rng_r, t.finish.has_finish ? &fin_r : nullptr);
    }
    std::printf("  %-34s %5ld ticks identical (%s)\n", name, k, !u.alive ? "crashed" : u.finished ? "finished" : "running");
    return k;
}

static void TestDistributions() {
    std::mt19937 a(123), b(123);
    std::uniform_real_distribution<float> U(0.0f, 0.37f);
    for (int i = 0; i < 200000; ++i) { float x = U(a), y = rt::Uniform(b, 0.0f, 0.37f); if (!Same(x, y)) { CHECK(false, "uniform draw %d", i); return; } }
    std::normal_distribution<float> N(0.1f, 0.02f);
    rt::Normal M(0.1f, 0.02f);
    for (int i = 0; i < 200000; ++i) { float x = N(a), y = M(b); if (!Same(x, y)) { CHECK(false, "normal draw %d", i); return; } }
    for (int i = 0; i < 50000; ++i) {
        std::normal_distribution<float> n1(0.0f, 0.004f);
        float x = n1(a), y = rt::NormalOnce(b, 0.0f, 0.004f);
        if (!Same(x, y)) { CHECK(false, "fresh normal draw %d", i); return; }
    }
    std::printf("  distributions: uniform / normal / fresh normal identical to libstdc++\n");
}

int main() {
    std::printf("Upstream equivalence (vendor/simulation-to-real-life @ %s)\n", "SOURCE_COMMIT");
    TestDistributions();
    SimConfig base = RealRobot();
    RunPair("real robot, w20_s", base, "w20_s", 1, 20000, false, false, true);
    RunPair("real robot, w20_s, wall jitter", base, "w20_s", 2, 20000, true, false, true);
    RunPair("real robot, random spawn", base, "w20_err_A", 3, 20000, true, true, true);
    RunPair("real robot, no culling", base, "c90_l_20", 4, 20000, true, true, false);
    for (unsigned s = 10; s < 16; ++s) {
        char nm[64]; std::snprintf(nm, sizeof nm, "real robot, seed %u", s);
        RunPair(nm, base, s % 2 ? "c90_l_20" : "w20_err_A", s, 20000, true, true, true);
    }
    SimConfig simple = base;
    simple.wheel_model = 0; simple.encoder_model = 0; simple.tof_latency_s = 0; simple.tof_delay_steps = 2;
    simple.emerg_on = 1; simple.motor_tau_s = 0.05f; simple.motor_max_accel_mps2 = 1.0f; simple.control_period_s = 0.05f;
    RunPair("simple wheels, delay 2, emergency", simple, "w20_s", 5, 20000, true, false, true);
    SimConfig wide = base;
    wide.tof_rays = 7; wide.tof_noise_pct_of_range = 0.03f; wide.steer_smooth = 0; wide.pid_base_speed_frac = 0.6f;
    RunPair("7 rays, no smoothing, 60% speed", wide, "n_straight_short", 6, 20000, true, true, true);
    std::printf(g_fail ? "FAILED (%d)\n" : "OK: the engine reproduces the upstream simulator bit for bit\n", g_fail);
    return g_fail ? 1 : 0;
}
