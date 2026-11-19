// golden.cpp - native reference run of the engine through its C API (sim/rt_api.cpp, the file compiled to WASM).
//
//   golden --write   writes tests/golden/scenarios.txt (scenario definitions, from upstream tracks) and
//                    tests/golden/golden.txt (a hash of every tick's state of every scenario)
//   golden --check   re-runs scenarios.txt and compares with golden.txt
//
// tests/wasm_equiv.test.mjs replays scenarios.txt through web/wasm/rt_sim.wasm and must reproduce golden.txt.
// Both sides read the SAME text file, so they start from the same numbers.
#include "../../sim/rt_api.cpp"
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>

struct Scenario {
    std::string name, config;
    unsigned seed = 1; int ticks = 100; float sx = 0, sy = 0, sh = 0; int external = 0, sample_bias = 1;
    std::vector<float> walls, center; int finish = 0; float fin[4] = {0, 0, 0, 0};
    unsigned eval_seed = 0; int eval_gen = 0, eval_eps = 0;
};

static uint32_t Fnv(uint32_t h, const void* p, size_t n) {
    const unsigned char* b = (const unsigned char*)p;
    for (size_t i = 0; i < n; ++i) { h ^= b[i]; h *= 16777619u; }
    return h;
}

static std::vector<Scenario> Read(const char* path) {
    std::ifstream f(path);
    std::vector<Scenario> out;
    std::string line;
    Scenario* s = nullptr;
    while (std::getline(f, line)) {
        std::istringstream ss(line);
        std::string k; ss >> k;
        if (k == "scenario") { out.emplace_back(); s = &out.back(); ss >> s->name; }
        else if (!s) continue;
        else if (k == "seed") ss >> s->seed;
        else if (k == "ticks") ss >> s->ticks;
        else if (k == "external") ss >> s->external;
        else if (k == "sample_bias") ss >> s->sample_bias;
        else if (k == "spawn") { std::string a, b, c; ss >> a >> b >> c; s->sx = std::strtof(a.c_str(), 0); s->sy = std::strtof(b.c_str(), 0); s->sh = std::strtof(c.c_str(), 0); }
        else if (k == "config") { while (std::getline(f, line) && line != "end") s->config += line + "\n"; }
        else if (k == "walls" || k == "center") {
            int n; ss >> n; int per = k == "walls" ? 4 : 3;
            for (int i = 0; i < n && std::getline(f, line); ++i) {
                std::istringstream ls(line);
                for (int j = 0; j < per; ++j) { std::string v; ls >> v; (k == "walls" ? s->walls : s->center).push_back(std::strtof(v.c_str(), 0)); }
            }
        }
        else if (k == "finish") { ss >> s->finish; for (int j = 0; j < 4; ++j) { std::string v; ss >> v; s->fin[j] = std::strtof(v.c_str(), 0); } }
        else if (k == "evaluate") ss >> s->eval_seed >> s->eval_gen >> s->eval_eps;
    }
    return out;
}

// Must match tests/wasm_equiv.test.mjs exactly.
static std::string Run(const Scenario& s) {
    int id = rt_create();
    rt_config(id, s.config.c_str());
    rt_walls_set(id, s.walls.data(), (int)s.walls.size() / 4);
    if (!s.center.empty()) rt_centerline_set(id, s.center.data(), (int)s.center.size() / 3);
    rt_finish_set(id, s.finish, s.fin[0], s.fin[1], s.fin[2], s.fin[3]);
    rt_spawn_set(id, s.sx, s.sy, s.sh);
    rt_reset(id, s.sx, s.sy, s.sh, s.seed, s.sample_bias);
    rt_set_external(id, s.external);
    uint32_t h = 2166136261u;
    int k = 0;
    for (; k < s.ticks; ++k) {
        if (s.external) {
            float vl = (float)(0.06 + 0.01 * (k % 7)), vr = (float)(0.06 + 0.01 * ((k * 3) % 7));
            if ((k / 40) % 3 == 2) vl = -vl;
            rt_ext_cmd(id, vl, vr);
        }
        int alive = rt_step(id, 1);
        float* st = rt_state(id);
        static const int idx[] = {1, 2, 3, 4, 5, 9, 10, 15, 24, 25, 27, 33, 34, 35, 36, 37};
        for (int i : idx) h = Fnv(h, &st[i], 4);
        if (!alive || st[7] != 0.0f) { ++k; break; }
    }
    char buf[256];
    std::snprintf(buf, sizeof buf, "%s ticks %d hash %08x", s.name.c_str(), k, h);
    std::string out = buf;
    for (int e = 0; e < s.eval_eps; ++e) {
        float* r = rt_evaluate(id, s.eval_seed, s.eval_gen, e);
        uint32_t he = Fnv(2166136261u, r, 9 * 4);
        std::snprintf(buf, sizeof buf, " ep%d %08x", e, he);
        out += buf;
    }
    rt_destroy(id);
    return out;
}

// ---- scenario generation from upstream track files -------------------------------------------------
static void LoadTrack(const std::string& path, Scenario& s) {
    std::ifstream f(path);
    std::string line;
    std::vector<std::string> w, c;
    while (std::getline(f, line)) {
        std::istringstream ss(line); std::string k; ss >> k;
        if (k == "wall") w.push_back(line.substr(5));
        else if (k == "center") c.push_back(line.substr(7));
        else if (k == "spawn") { float hd; ss >> s.sx >> s.sy >> hd; s.sh = hd * 3.14159265359f / 180.0f; }
        else if (k == "finish") { s.finish = 1; ss >> s.fin[0] >> s.fin[1] >> s.fin[2] >> s.fin[3]; }
    }
    for (auto& x : w) { std::istringstream ss(x); for (int j = 0; j < 4; ++j) { float v; ss >> v; s.walls.push_back(v); } }
    for (auto& x : c) { std::istringstream ss(x); for (int j = 0; j < 3; ++j) { float v; ss >> v; s.center.push_back(v); } }
}

static const char* REAL = R"(dt 0.025
track_width_m 0.1249
wheel_radius_m 0.0225
max_linear_speed_mps 0.156
chassis_front_m 0.14
chassis_rear_m 0.04
chassis_width_m 0.14
wheel_offset_from_center_m 0
wheel_length_m 0.045
wheel_width_m 0.02
n_sensors 3
sensor_0_pose 0.10076 -0.01024 -45
sensor_1_pose 0.105 0 0
sensor_2_pose 0.09864 0.00812 45
enc_ticks_per_rev 8291.5
pwm_limit 255
wheel_pid_period_ms 10
motor_k_l 0.000613
motor_k_r 0.000618
motor_static_l 3.2
motor_static_r 3.8
motor_breakaway_l 20
motor_breakaway_r 20
motor_tau_l_s 0.0362
motor_tau_r_s 0.04
motor_nominal_v 8.05
wheel_kp 2
wheel_ki 0.001
wheel_kd 5
wheel_ff_gain 2.785
wheel_ff_static 4
tof_glitch_threshold_m 0.12
tof_glitch_max_hold_s 2
encoder_model 1
motor_tau_s 0.1
tof_latency_s 0.0132
tof_latency_jitter_s 0.018
wheel_model 1
battery_v 8.05
motor_mismatch_std 0.02
sensor_angle_bias_std_deg 2
wheel_scale_bias_std 0.02
max_speed_bias_std 0.05
emerg_on 0
steer_turn_accel 1.5
steer_max_step 0.0105
kp 0.586855
ki 0.0158843
kd 0.384207
base_speed_frac 1
pid_error_clip_m 0.08
wall_noise_std_m 0.01
max_episode_s 30
spawn_heading_random_deg 30
spawn_lateral_random_frac 0.5
spawn_margin_m 0.02
speed_bonus 800
center_weight 10
smooth_weight 20
)";

static std::string Sensors(int n) {
    std::string s;
    for (int i = 0; i < 3; ++i) {
        char b[256];
        std::snprintf(b, sizeof b, "sensor_%d_half_angle_rad 0.16816\nsensor_%d_max_range_m 1.3\nsensor_%d_min_range_m 0.04\nsensor_%d_noise_std_m 0.004\nsensor_%d_dropout 0.02\n", i, i, i, i, i);
        s += b;
    }
    (void)n;
    return s;
}

static void Write(const char* scen, const char* gold) {
    const std::string V = "vendor/simulation-to-real-life/tracks/";
    std::vector<Scenario> v;
    auto add = [&](const char* name, const char* track, std::string cfg, unsigned seed, int ticks, int ext, unsigned es, int eg, int ee) {
        Scenario s; s.name = name; LoadTrack(V + track + ".track", s); s.config = cfg; s.seed = seed; s.ticks = ticks; s.external = ext;
        s.eval_seed = es; s.eval_gen = eg; s.eval_eps = ee; v.push_back(s);
    };
    std::string real = std::string(REAL) + Sensors(3);
    add("real_w20_s", "w20_s", real, 1, 4000, 0, 7, 0, 4);
    add("real_err_A", "w20_err_A", real, 2, 4000, 0, 9, 3, 4);
    add("real_c90", "c90_l_20", real + "kp 0.3\nkd 0.2\nbase_speed_frac 0.7\n", 3, 4000, 0, 0, 0, 0);
    add("simple_wheels", "n_straight_short", real + "wheel_model 0\nencoder_model 0\ntof_delay_steps 2\nemerg_on 1\nmotor_max_accel_mps2 1\n", 4, 4000, 0, 0, 0, 0);
    std::string five = real + "n_sensors 5\nsensor_3_pose 0.08 0.05 90\nsensor_4_pose 0.08 -0.05 -90\nsensor_3_model 1\nsensor_3_half_angle_rad 0.03\n"
        "sensor_3_rays 1\nsensor_3_max_range_m 0.8\nsensor_3_min_range_m 0.1\nsensor_3_noise_pct 0.03\nsensor_4_model 2\nsensor_4_half_angle_rad 0.26\n"
        "sensor_4_rays 9\nsensor_4_max_range_m 4\nsensor_4_min_range_m 0.02\nsensor_4_max_incidence_deg 40\nsensor_4_noise_std_m 0.003\n"
        "four_wheel 1\nwheel_base_m 0.08\nwheel_track_geom_m 0.12\nctrl_track_width_m 0.13\n";
    add("five_sensors_4wd", "w20_s", five, 5, 4000, 0, 11, 1, 2);
    add("one_side", "w20_err_A", real + "n_sensors 2\nsensor_1_pose 0.09864 0.00812 45\npid_mode 1\nfollow_side 1\nfollow_dist_m 0.12\n", 6, 4000, 0, 0, 0, 0);
    add("external", "w20_s", real + "steer_smooth 0\n", 7, 600, 1, 0, 0, 0);
    std::ofstream o(scen);
    o << "# Generated by tests/native/golden.cpp --write. Do not edit by hand.\n";
    char b[128];
    for (auto& s : v) {
        o << "scenario " << s.name << "\nseed " << s.seed << "\nticks " << s.ticks << "\nexternal " << s.external << "\nsample_bias 1\n";
        std::snprintf(b, sizeof b, "spawn %.9g %.9g %.9g\n", s.sx, s.sy, s.sh); o << b;
        o << "config\n" << s.config << "end\n";
        o << "walls " << s.walls.size() / 4 << "\n";
        for (size_t i = 0; i < s.walls.size(); i += 4) { std::snprintf(b, sizeof b, "%.9g %.9g %.9g %.9g\n", s.walls[i], s.walls[i + 1], s.walls[i + 2], s.walls[i + 3]); o << b; }
        o << "center " << s.center.size() / 3 << "\n";
        for (size_t i = 0; i < s.center.size(); i += 3) { std::snprintf(b, sizeof b, "%.9g %.9g %.9g\n", s.center[i], s.center[i + 1], s.center[i + 2]); o << b; }
        std::snprintf(b, sizeof b, "finish %d %.9g %.9g %.9g %.9g\n", s.finish, s.fin[0], s.fin[1], s.fin[2], s.fin[3]); o << b;
        o << "evaluate " << s.eval_seed << " " << s.eval_gen << " " << s.eval_eps << "\n";
    }
    o.close();
    std::ofstream g(gold);
    for (auto& s : Read(scen)) { std::string r = Run(s); g << r << "\n"; std::printf("%s\n", r.c_str()); }
}

int main(int argc, char** argv) {
    const char* scen = "tests/golden/scenarios.txt";
    const char* gold = "tests/golden/golden.txt";
    if (argc > 1 && std::string(argv[1]) == "--write") { Write(scen, gold); return 0; }
    std::ifstream g(gold);
    std::string line; int bad = 0, n = 0;
    for (auto& s : Read(scen)) {
        std::getline(g, line);
        std::string r = Run(s); ++n;
        if (r != line) { ++bad; std::printf("DIFF\n  got  %s\n  want %s\n", r.c_str(), line.c_str()); }
    }
    std::printf(bad ? "golden: %d of %d scenarios differ\n" : "golden: %d scenarios identical\n", bad ? bad : n, n);
    return bad ? 1 : 0;
}
