// rt_api.cpp - C API of the engine, exported to JavaScript (WebAssembly) and used by the native tests.
// Every function takes an instance id from rt_create(). Arrays are passed as pointers into the module's memory.
// The layout of rt_state() is documented in web/js/engine.js (STATE_*), keep the two in sync.
#include "rt_engine.hpp"
#include <memory>
#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>
#define API extern "C" EMSCRIPTEN_KEEPALIVE
#else
#define API extern "C"
#endif

using namespace rt;

namespace {
struct Inst {
    Sim sim;
    Track track;
    Rng rng{1};
    int hint = -1;
    float state[64];
    float rays[MAXS * 6];
    float foot[(1 + 4) * 8];
    float stats[16];
    float spawn[3];
};
std::vector<std::unique_ptr<Inst>> g_inst;
Inst* I(int id) { return id >= 0 && id < (int)g_inst.size() ? g_inst[id].get() : nullptr; }
}  // namespace

API int rt_version() { return 1; }

API int rt_create() {
    for (size_t i = 0; i < g_inst.size(); ++i)
        if (!g_inst[i]) { g_inst[i].reset(new Inst()); return (int)i; }
    g_inst.emplace_back(new Inst());
    return (int)g_inst.size() - 1;
}
API void rt_destroy(int id) { if (I(id)) g_inst[id].reset(); }

// Config text ("key value" lines). Merged into the current config; the PID gains / sensors take effect at once,
// without clearing the controller state (live sliders). Returns the number of unknown keys.
API int rt_config(int id, const char* text) {
    Inst* p = I(id); if (!p) return -1;
    int u = ParseConfig(p->sim.cfg, text);
    float integ = p->sim.pid.integral, prev = p->sim.pid.prev_err, df = p->sim.pid.d_filtered; bool hp = p->sim.pid.has_prev;
    p->sim.pid.Configure(p->sim.cfg);
    p->sim.pid.integral = integ; p->sim.pid.prev_err = prev; p->sim.pid.d_filtered = df; p->sim.pid.has_prev = hp;
    return u;
}
API void rt_config_reset(int id) { Inst* p = I(id); if (p) p->sim.cfg = Config(); }

// Walls: n walls as x1 y1 x2 y2.
API void rt_walls_set(int id, const float* xy, int n) {
    Inst* p = I(id); if (!p) return;
    p->track.walls.resize(n);
    for (int i = 0; i < n; ++i) p->track.walls[i] = {{xy[4 * i], xy[4 * i + 1]}, {xy[4 * i + 2], xy[4 * i + 3]}};
    p->sim.walls = p->track.walls;
}
// Centre line for the guide: n points as x y half_width.
API void rt_centerline_set(int id, const float* p3, int n) {
    Inst* p = I(id); if (!p) return;
    std::vector<V2> pts(n); std::vector<float> hw(n);
    for (int i = 0; i < n; ++i) { pts[i] = {p3[3 * i], p3[3 * i + 1]}; hw[i] = p3[3 * i + 2]; }
    p->track.guide.Build(pts, hw);
    p->hint = -1;
}
API void rt_finish_set(int id, int on, float x1, float y1, float x2, float y2) {
    Inst* p = I(id); if (!p) return;
    p->track.has_finish = on != 0; p->track.finish = {{x1, y1}, {x2, y2}};
}
API void rt_spawn_set(int id, float x, float y, float heading) {
    Inst* p = I(id); if (!p) return;
    p->track.spawn = {x, y}; p->track.spawn_heading = heading;
}
// Random start pose (upstream RandomSpawn) against the current walls. Returns pointer to x, y, heading.
API float* rt_random_spawn(int id, unsigned seed) {
    Inst* p = I(id); if (!p) return nullptr;
    Rng r(seed);
    V2 pos; float h;
    Sim probe; probe.cfg = p->sim.cfg; probe.walls = p->sim.walls; probe.Reset(p->track.spawn, p->track.spawn_heading);
    RandomSpawn(probe, p->track.guide, p->track.spawn, p->track.spawn_heading, r, pos, h);
    p->spawn[0] = pos.x; p->spawn[1] = pos.y; p->spawn[2] = h;
    return p->spawn;
}
// Start a run: pose, the run's random seed, and whether to draw this run's hardware bias.
API void rt_reset(int id, float x, float y, float heading, unsigned seed, int sample_bias) {
    Inst* p = I(id); if (!p) return;
    p->rng.seed(seed);
    p->sim.Reset({x, y}, heading);
    if (sample_bias) p->sim.SampleHardwareBias(p->rng);
    p->hint = -1;
}
API void rt_set_external(int id, int on) { Inst* p = I(id); if (p) p->sim.external = on != 0; }
API void rt_ext_cmd(int id, float vl, float vr) { Inst* p = I(id); if (p) { p->sim.ext_l = vl; p->sim.ext_r = vr; } }
API void rt_set_cull(int id, int on) { Inst* p = I(id); if (p) p->sim.cull = on != 0; }

// Advance n control ticks (stops early when the robot crashes or finishes). Returns alive.
API int rt_step(int id, int n) {
    Inst* p = I(id); if (!p) return 0;
    const Wall* fin = p->track.has_finish ? &p->track.finish : nullptr;
    for (int k = 0; k < n && p->sim.alive && !p->sim.finished; ++k) p->sim.Step(p->rng, fin);
    return p->sim.alive ? 1 : 0;
}

API float* rt_state(int id) {
    Inst* p = I(id); if (!p) return nullptr;
    Sim& s = p->sim; float* o = p->state;
    std::fill(o, o + 64, 0.0f);
    o[0] = s.steps * s.cfg.dt; o[1] = s.pos.x; o[2] = s.pos.y; o[3] = s.theta; o[4] = s.v_l; o[5] = s.v_r;
    o[6] = s.alive; o[7] = s.finished; o[8] = (float)s.state; o[9] = s.enc_l; o[10] = s.enc_r;
    o[11] = s.wheel_l; o[12] = s.wheel_r; o[13] = s.last_eff_l; o[14] = s.last_eff_r;
    o[15] = s.pid.err; o[16] = s.pid.p_term; o[17] = s.pid.i_term; o[18] = s.pid.d_term; o[19] = s.pid.turn; o[20] = s.pid.base;
    o[21] = s.pid.sat; o[22] = (float)s.drive.pwm[0]; o[23] = (float)s.drive.pwm[1];
    o[24] = (float)s.enc_dist_l; o[25] = (float)s.enc_dist_r;
    if (p->track.guide.valid) {
        Guide::Query q = p->track.guide.At(s.pos, s.theta, &p->hint);
        o[26] = q.s; o[27] = q.center_err; o[28] = q.heading_err; o[29] = q.half_width; o[30] = q.ok;
    }
    o[31] = (float)s.pid.left_idx; o[32] = (float)s.pid.right_idx;
    for (int i = 0; i < MAXS; ++i) { o[33 + i] = s.dists[i]; o[38 + i] = s.raw[i]; }
    o[43] = (float)s.steps;
    o[44] = (float)s.drive.Ticks(0, s.cfg); o[45] = (float)s.drive.Ticks(1, s.cfg);
    o[46] = s.emergency.count; o[47] = s.wheel_scale_l; o[48] = s.wheel_scale_r;
    o[49] = s.motor_mismatch[0]; o[50] = s.motor_mismatch[1];
    for (int i = 0; i < MAXS; ++i) o[51 + i] = s.sensor_angle_bias_deg[i];
    return o;
}

// Per sensor (current pose): origin x, y, world angle (incl. this run's mounting error), filtered reading,
// half cone, max range.
API float* rt_rays(int id) {
    Inst* p = I(id); if (!p) return nullptr;
    Sim& s = p->sim;
    for (int i = 0; i < s.cfg.n_sensors; ++i) {
        const Sensor& m = s.cfg.sensors[i];
        float* r = p->rays + 6 * i;
        r[0] = s.pos.x + Cos(s.theta) * m.x_m - Sin(s.theta) * m.y_m;
        r[1] = s.pos.y + Sin(s.theta) * m.x_m + Cos(s.theta) * m.y_m;
        r[2] = s.theta + (m.angle_deg + s.sensor_angle_bias_deg[i]) * PI / 180.0f;
        r[3] = s.dists[i]; r[4] = m.half_angle_rad; r[5] = m.max_range_m;
    }
    return p->rays;
}

// Footprint: body corners then each wheel's corners (x y pairs), 2 or 4 wheels. Returns pointer; count = 1 + wheels.
API float* rt_footprint(int id) {
    Inst* p = I(id); if (!p) return nullptr;
    V2 c[4];
    p->sim.BodyCorners(c);
    for (int k = 0; k < 4; ++k) { p->foot[2 * k] = c[k].x; p->foot[2 * k + 1] = c[k].y; }
    for (int w = 0; w < p->sim.NumWheels(); ++w) {
        p->sim.WheelCorners(w, c);
        for (int k = 0; k < 4; ++k) { p->foot[8 * (w + 1) + 2 * k] = c[k].x; p->foot[8 * (w + 1) + 2 * k + 1] = c[k].y; }
    }
    return p->foot;
}

// One training episode on this instance's track (upstream EvaluateRobot): score, crashed, finished, steps,
// forward_m, mean |centre err|, rms centre err, mean |heading err|, time_s.
API float* rt_evaluate(int id, unsigned run_seed, int generation, int episode) {
    Inst* p = I(id); if (!p) return nullptr;
    EpisodeStats st = Evaluate(p->sim.cfg, p->track, run_seed, generation, episode);
    float* o = p->stats;
    o[0] = st.score; o[1] = st.crashed; o[2] = st.finished; o[3] = st.steps; o[4] = st.forward_m;
    o[5] = st.mean_abs_center; o[6] = st.rms_center; o[7] = st.mean_abs_heading; o[8] = st.time_s;
    return o;
}
