// record: run one episode headless and save it as an interactive replay (a single .html file) in videos/.
//
// The replay plays back the run in any browser: zoom, pan, scrub, change speed, and switch layers on and
// off -- including the walls, so you can watch the robot drive purely on what its sensors reported. Each
// ToF reading is drawn as a coloured dot where it hit; the dots build up into the map the robot "saw".
//
// The episode is the same one the visualizer shows: view_track from sim_config.txt, unperturbed walls,
// rng seed 1234, a freshly sampled hardware bias. Works with whichever controller this folder builds
// (NN, PID or LineFit): the brain type comes from Robot.hpp.
//
//   record [--track NAME] [--model PATH] [--seed N] [--out FILE] [--open]
//          [--top-speed V]            forward top speed of the recorded run (m/s; default: the trained 0.156)
//          [--spawn x,y,deg]          start pose (default: the track file's spawn)
//          [--resim LIST]             EXPERIMENTAL "what if faster": also re-simulate the same run (same track, start pose,
//                                     seed) with the top speed x each multiplier, capped at 0.156 m/s; "auto" = x1.0,
//                                     x1.25, x1.5, ... up to 0.156
//          [--ramp]                   ... and once with a ramp from the original speed to 0.156 m/s over the run
//          [--resim-json FILE]        write only the re-simulations as JSON (tools/log_to_replay.py, real runs)
//   (viz rep on Windows; make replay on Linux)
#include "Types.hpp"
#include "Raycast.hpp"
#include "Track.hpp"
#include "Robot.hpp"
#include "SimConfig.hpp"
#include <algorithm>
#include <cstdio>
#include <chrono>
#include <cstdlib>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

namespace fs = std::filesystem;
using Brain = decltype(Robot::brain);

// ---------------------------------------------------------------- controller-specific extras (optional)
// A brain with fitted wall lines (LineFit) gets them recorded.
template <class B, class = void> struct HasLines : std::false_type {};
template <class B> struct HasLines<B, std::void_t<decltype(std::declval<B>().left_line.ok)>> : std::true_type {};

template <class B, class = void> struct HasWeights : std::false_type {};
template <class B> struct HasWeights<B, std::void_t<decltype(std::declval<B>().w1)>> : std::true_type {};
template <class B, class = void> struct HasGains : std::false_type {};
template <class B> struct HasGains<B, std::void_t<decltype(std::declval<B>().kp)>> : std::true_type {};

template <class B, class = void> struct HasMapMem : std::false_type {};
template <class B> struct HasMapMem<B, std::void_t<decltype(std::declval<B>().mem)>> : std::true_type {};

static std::string ControllerName() {
    if constexpr (HasLines<Brain>::value) return "LineFit";
    else if constexpr (HasGains<Brain>::value) return "PID";
    else if constexpr (HasWeights<Brain>::value) return "NN";
    else if constexpr (HasMapMem<Brain>::value) return "NN-map";
    else return "controller";
}

static std::string Num(float v);

// LineFit's lines live in its own odometry frame, which drifts. They are written relative to the robot
// (that is what it steers on) and placed on the robot's true pose.
template <class B> void WriteLines(std::ostream& o, const B& b, float px, float py, float pth) {
    if constexpr (HasLines<B>::value) {
        for (const auto* L : {&b.left_line, &b.right_line}) {
            if (!L->ok) { o << ",null,null,null,null"; continue; }
            float h = 0.6f * L->spread + 0.3f;
            for (int e = -1; e <= 1; e += 2) {
                float wx = L->cx + e * L->dx * h - b.ox, wy = L->cy + e * L->dy * h - b.oy;
                float c = std::cos(-b.oth), s = std::sin(-b.oth);
                float lx = c * wx - s * wy, ly = s * wx + c * wy;  // in the robot's own frame
                float C = std::cos(pth), S = std::sin(pth);
                o << "," << Num(px + C * lx - S * ly) << "," << Num(py + S * lx + C * ly);
            }
        }
    } else { (void)o; (void)b; (void)px; (void)py; (void)pth; }
}

// LineFit's OWN window points (the ones each line was fitted from this tick), placed exactly like the lines:
// odometry frame -> robot frame -> the robot's true pose. Then each line's rms to its own points (m).
// Row fields 31/32: flat [x,y,x,y,...] left / right, 33/34: rms left / right (null without a line),
// 35/36: window points dropped by the outlier step, left / right (tasks.txt A4).
template <class B> void WriteFitPoints(std::ostream& o, const B& b, float px, float py, float pth) {
    if constexpr (HasLines<B>::value) {
        float c = std::cos(-b.oth), s = std::sin(-b.oth), C = std::cos(pth), S = std::sin(pth);
        for (const auto* pts : {&b.used_l, &b.used_r}) {
            o << ",[";
            bool first = true;
            for (const auto& p : *pts) {
                float wx = p.x - b.ox, wy = p.y - b.oy;
                float lx = c * wx - s * wy, ly = s * wx + c * wy;
                o << (first ? "" : ",") << Num(px + C * lx - S * ly) << "," << Num(py + S * lx + C * ly);
                first = false;
            }
            o << "]";
        }
        for (const auto* L : {&b.left_line, &b.right_line}) o << "," << (L->ok ? Num(L->rms) : std::string("null"));
        // 35/36: the window points of each side that the outlier step DROPPED (in the window, not in the fit).
        const std::vector<typename B::Pt>* sides[2] = {&b.side_l, &b.side_r};
        const std::vector<typename B::Pt>* used[2] = {&b.used_l, &b.used_r};
        for (int j = 0; j < 2; ++j) {
            o << ",[";
            bool first = true;
            for (const auto& p : *sides[j]) {
                bool in = false;
                for (const auto& u : *used[j]) if (u.seq == p.seq) { in = true; break; }
                if (in) continue;
                float wx = p.x - b.ox, wy = p.y - b.oy;
                float lx = c * wx - s * wy, ly = s * wx + c * wy;
                o << (first ? "" : ",") << Num(px + C * lx - S * ly) << "," << Num(py + S * lx + C * ly);
                first = false;
            }
            o << "]";
        }
    } else { (void)o; (void)b; (void)px; (void)py; (void)pth; }
}

// ------------------------------------------------------------------ ground-truth telemetry helpers
static float SegDist(Vec2 a, Vec2 b, Vec2 c, Vec2 d);
static float PointSegDist(Vec2 p, Vec2 a, Vec2 b) {
    float vx = b.x - a.x, vy = b.y - a.y, L2 = vx * vx + vy * vy;
    float t = L2 > 0.0f ? std::clamp(((p.x - a.x) * vx + (p.y - a.y) * vy) / L2, 0.0f, 1.0f) : 0.0f;
    float dx = a.x + t * vx - p.x, dy = a.y + t * vy - p.y;
    return std::sqrt(dx * dx + dy * dy);
}
static float SegDist(Vec2 a, Vec2 b, Vec2 c, Vec2 d) {
    if (Robot::SegmentsIntersect(a, b, c, d)) return 0.0f;
    return std::min({PointSegDist(a, c, d), PointSegDist(b, c, d), PointSegDist(c, a, b), PointSegDist(d, a, b)});
}

// Closest distance from the robot's footprint (chassis + both wheels) to any wall.
static float Clearance(const Robot& r, const SimConfig& cfg, const std::vector<Wall>& walls) {
    Vec2 parts[3][4];
    r.ChassisCorners(cfg, parts[0]);
    r.WheelCorners(cfg, 0, parts[1]);
    r.WheelCorners(cfg, 1, parts[2]);
    float best = 1e9f;
    for (const auto& w : walls)
        for (const auto& c : parts)
            for (int i = 0; i < 4; ++i) best = std::min(best, SegDist(c[i], c[(i + 1) % 4], w.p1, w.p2));
    return best;
}

// Heading and centring error against the LOCAL corridor: the axis is the direction of the nearest wall
// (pointed the way the robot faces), the centre is halfway between the walls found perpendicular to it.
// Works on straight tracks and on curves alike. Exact geometry, no sensor noise.
static void CorridorErrors(Vec2 p, float th, const std::vector<Wall>& walls, float& heading_err_deg, float& centre_err_m) {
    const Wall* nearest = nullptr;
    float bd = 1e9f;
    for (const auto& w : walls) {
        float d = PointSegDist(p, w.p1, w.p2);
        if (d < bd) { bd = d; nearest = &w; }
    }
    heading_err_deg = 0.0f;
    centre_err_m = 0.0f;
    if (!nearest) return;
    float axis = std::atan2(nearest->p2.y - nearest->p1.y, nearest->p2.x - nearest->p1.x);
    auto wrap = [](float a) { while (a > PI_F) a -= 2 * PI_F; while (a <= -PI_F) a += 2 * PI_F; return a; };
    if (std::abs(wrap(axis - th)) > 0.5f * PI_F) axis = wrap(axis + PI_F);
    heading_err_deg = wrap(th - axis) * 180.0f / PI_F;
    // Screen y points down, so +90 deg from the axis is the robot's RIGHT and -90 deg its LEFT (the same
    // convention as the replay's sensor names). + = more room on the right = robot is toward the left wall.
    const float LOOK = 1.5f;
    float room_right = CastSingleRay(p, axis + 0.5f * PI_F, walls, LOOK);
    float room_left = CastSingleRay(p, axis - 0.5f * PI_F, walls, LOOK);
    // Opposite a gap in a wall there is no centre to be off from: no value.
    centre_err_m = (room_right >= LOOK - 1e-3f || room_left >= LOOK - 1e-3f) ? NAN : 0.5f * (room_right - room_left);
}

static const char* DefaultModels[] = {"best_robot_model.txt", "best_pid_model.txt", "best_linefit_model.txt", "best_nnmap_model.txt"};

struct Args {
    std::string track, model, out, resim, resim_json, spawn;
    unsigned seed = 1234;
    bool open = false, ramp = false;
    float top = 0.0f;
};

// ------------------------------------------------------------ "what if faster" re-simulation (JOB 8, experimental)
// The same episode (track, start pose, rng seed, hardware draw) driven again with another forward top speed
// (sim key exp_top_speed_mps, see SimConfig.hpp). Every controller takes it: PID / LineFit cruise, NN-map v output,
// the old NN both wheel outputs (Robot.hpp reads cfg each tick).
template <class B, class = void> struct HasTop : std::false_type {};
template <class B> struct HasTop<B, std::void_t<decltype(std::declval<B>().top_speed)>> : std::true_type {};
template <class B, class = void> struct HasMapP : std::false_type {};
template <class B> struct HasMapP<B, std::void_t<decltype(std::declval<B>().P.v_top)>> : std::true_type {};
template <class B> static void SetTopOf(B& b, float v) {
    if constexpr (HasTop<B>::value) b.top_speed = v;
    if constexpr (HasMapP<B>::value) b.P.v_top = v;
    (void)b; (void)v;
}
static void SetTopSpeed(Robot& bot, SimConfig& cfg, float v) {
    cfg.exp_top_speed_mps = v;
    SetTopOf(bot.brain, v);
}

struct SimTick { float t, x, y, th, s, v; };
struct SimRun { std::string label, kind, outcome; float top = 0; std::vector<SimTick> ticks; };

// One episode; top(t) = forward top speed at time t (<= 0: the trained speed). s = distance along the track (guide).
template <class TopFn>
static SimRun Simulate(SimConfig cfg, const LoadedTrack& t, const Brain& proto, unsigned seed, Vec2 sp, float sh, TopFn top) {
    SimRun r;
    Robot bot;
    bot.brain = proto;
    std::mt19937 rng(seed);
    bot.Reset(sp, sh, cfg.tof_max_range_m);
    bot.SampleHardwareBias(cfg, rng);
    const Wall* finish = t.finish.has_finish ? &t.finish.segment : nullptr;
    r.outcome = "TIMED OUT";
    float s_last = 0.0f;
    for (int k = 0; k < cfg.MaxSteps(); ++k) {
        float tt = k * SIMULATION_DT;
        float v = top(tt);
        if (v > 0.0f) SetTopSpeed(bot, cfg, v);
        StraightGuide::Query q = t.guide.At(bot.pos, bot.theta);
        if (q.ok) s_last = q.s;
        Vec2 p = bot.pos; float th = bot.theta;
        bot.Step(t.walls, cfg, rng, finish);
        r.ticks.push_back({tt, p.x, p.y, th, s_last, 0.5f * (bot.last_eff_v_l + bot.last_eff_v_r)});
        if (!bot.alive) { r.outcome = "CRASHED"; break; }
        if (bot.finished) { r.outcome = "FINISHED"; break; }
    }
    return r;
}

// JSON of the re-simulations, each with where it first leaves the original path by > 2 cm (compared at the same
// distance along the track) and where it crashed.
static std::string ResimJson(const std::vector<SimRun>& runs, const std::vector<SimTick>& orig) {
    std::ostringstream o;
    std::vector<float> smax(orig.size());
    for (size_t i = 0; i < orig.size(); ++i) smax[i] = std::max(i ? smax[i - 1] : -1e9f, orig[i].s);
    auto num = [](float v) { std::ostringstream q; q << std::setprecision(5) << (std::isfinite(v) ? v : 0.0f); return q.str(); };
    o << "[";
    for (size_t r = 0; r < runs.size(); ++r) {
        const SimRun& R = runs[r];
        int split = -1;
        for (size_t j = 0; j < R.ticks.size() && split < 0 && !orig.empty(); ++j) {
            size_t k = std::lower_bound(smax.begin(), smax.end(), R.ticks[j].s) - smax.begin();
            if (k >= orig.size()) break;  // past the end of the original
            if (std::hypot(R.ticks[j].x - orig[k].x, R.ticks[j].y - orig[k].y) > 0.02f) split = (int)j;
        }
        o << (r ? "," : "") << "{\"label\":\"" << R.label << "\",\"kind\":\"" << R.kind << "\",\"top\":" << num(R.top)
          << ",\"outcome\":\"" << R.outcome << "\",\"split\":";
        if (split >= 0) { const SimTick& q = R.ticks[split]; o << "{\"t\":" << num(q.t) << ",\"s\":" << num(q.s) << ",\"x\":" << num(q.x) << ",\"y\":" << num(q.y) << "}"; }
        else o << "null";
        o << ",\"ticks\":[";
        for (size_t j = 0; j < R.ticks.size(); ++j) {
            const SimTick& q = R.ticks[j];
            o << (j ? "," : "") << "[" << num(q.t) << "," << num(q.x) << "," << num(q.y) << "," << num(q.th) << "," << num(q.s) << "," << num(q.v) << "]";
        }
        o << "]}";
    }
    o << "]";
    return o.str();
}

static std::string Num(float v) {
    if (!std::isfinite(v)) return "null";
    std::ostringstream o;
    o << std::setprecision(5) << v;
    return o.str();
}

extern const char* kPlayerHtml;

int main(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        std::string k = argv[i];
        auto next = [&]() { return i + 1 < argc ? std::string(argv[++i]) : std::string(); };
        if (k == "--track") a.track = next();
        else if (k == "--model") a.model = next();
        else if (k == "--seed") a.seed = (unsigned)std::stoul(next());
        else if (k == "--out") a.out = next();
        else if (k == "--open") a.open = true;
        else if (k == "--top-speed") a.top = std::stof(next());
        else if (k == "--spawn") a.spawn = next();
        else if (k == "--resim") a.resim = next();
        else if (k == "--ramp") a.ramp = true;
        else if (k == "--resim-json") a.resim_json = next();
        else std::cerr << "Warning: unknown argument '" << k << "' ignored.\n";
    }

    SimConfig cfg = LoadConfig("sim_config.txt");
    if (a.track.empty()) a.track = cfg.view_track;
    LoadedTrack t = LoadNamedTrack(a.track);

    Robot bot;
    bot.brain = Brain(cfg);
    bool loaded = false;
    if (!a.model.empty()) {
        loaded = bot.brain.Load(a.model);
    } else {
        for (const char* m : DefaultModels) {
            std::ifstream probe(m);
            if (probe.good() && bot.brain.Load(m)) { a.model = m; loaded = true; break; }
        }
    }
    if (!loaded) std::cout << "Warning: no model loaded, recording the untrained controller.\n";

    Vec2 sp0 = Robot::SpawnAxle(cfg, t.spawn, t.spawn_heading);
    float sh0 = t.spawn_heading;
    if (!a.spawn.empty()) {  // x,y,deg (the axle position)
        float x = 0, y = 0, d = 0;
        if (std::sscanf(a.spawn.c_str(), "%f,%f,%f", &x, &y, &d) == 3) { sp0 = {x, y}; sh0 = d * PI_F / 180.0f; }
        else std::cerr << "Warning: --spawn wants x,y,deg\n";
    }
    const Brain proto = bot.brain;
    const float vmax = cfg.max_linear_speed_mps;
    const float v0 = a.top > 0.0f ? std::min(a.top, vmax) : vmax;
    SimConfig cfg_run = cfg;
    if (a.top > 0.0f) SetTopSpeed(bot, cfg_run, v0);

    // Re-simulations (experimental): same track, start pose and seed, other top speeds.
    std::vector<SimRun> resims;
    std::vector<SimTick> orig;
    if (!a.resim.empty() || a.ramp) {
        orig = Simulate(cfg, t, proto, a.seed, sp0, sh0, [&](float) { return v0; }).ticks;
        std::vector<float> mults;
        if (a.resim == "auto") { for (float k = 1.0f; v0 * k <= vmax * 1.0001f; k += 0.25f) mults.push_back(k); if (v0 * mults.back() < vmax * 0.999f) mults.push_back(vmax / v0); }
        else if (!a.resim.empty()) for (auto& m : SplitTrackList(a.resim)) mults.push_back(std::stof(m));
        for (float k : mults) {
            float v = std::min(vmax, v0 * k);
            char lab[96]; std::snprintf(lab, sizeof lab, "x%.2f (top %.3f m/s)%s", k, v, v0 * k > vmax * 1.0001f ? ", capped" : "");
            SimRun r = Simulate(cfg, t, proto, a.seed, sp0, sh0, [&](float) { return v; });
            r.label = lab; r.kind = "mult"; r.top = v;
            resims.push_back(r);
        }
        if (a.ramp) {
            float T = std::max(1.0f, orig.empty() ? 20.0f : orig.back().t);
            SimRun r = Simulate(cfg, t, proto, a.seed, sp0, sh0, [&](float tt) { return v0 + (vmax - v0) * std::min(1.0f, tt / T); });
            char lab[96]; std::snprintf(lab, sizeof lab, "ramp %.3f -> %.3f m/s over %.1f s", v0, vmax, T);
            r.label = lab; r.kind = "ramp"; r.top = vmax;
            resims.push_back(r);
        }
        for (const auto& r : resims) std::cout << "re-sim " << r.label << ": " << r.outcome << " after " << r.ticks.size() * SIMULATION_DT << " s\n";
        if (!a.resim_json.empty()) {
            std::ofstream jf(a.resim_json);
            jf << "{\"orig_top\":" << v0 << ",\"resims\":" << ResimJson(resims, orig) << "}";
            std::cout << "wrote " << a.resim_json << "\n";
            return 0;
        }
    }

    std::mt19937 rng(a.seed);
    bot.Reset(sp0, sh0, cfg.tof_max_range_m);
    bot.SampleHardwareBias(cfg, rng);
    const Wall* finish = t.finish.has_finish ? &t.finish.segment : nullptr;

    std::ostringstream ticks;
    std::string outcome = "TIMED OUT";
    int steps = 0;
    for (int s = 0; s < cfg.MaxSteps(); ++s) {
        // The ToF readings are taken at the start of Step, before it moves the robot, so the pose they
        // belong to is the one from before the call. Every row records that pose.
        const Vec2 pos = bot.pos;
        const float theta = bot.theta;
        Robot before = bot;  // footprint at the pose the readings belong to
        float clearance = Clearance(before, cfg, t.walls);
        float head_err = 0.0f, centre_err = 0.0f;
        CorridorErrors(pos, theta, t.walls, head_err, centre_err);
        bot.Step(t.walls, cfg_run, rng, finish);
        steps = s + 1;

        ticks << (s ? "," : "") << "[" << Num(s * SIMULATION_DT) << "," << Num(pos.x) << "," << Num(pos.y) << ","
              << Num(theta) << "," << Num(bot.v_l) << "," << Num(bot.v_r) << "," << (int)bot.state;
        for (int i = 0; i < 3; ++i) ticks << "," << Num(bot.dists[i]);
        // Hit points, with this robot's real (biased) sensor angles. A reading at max range is no return.
        for (int i = 0; i < 3; ++i) {
            const SensorMount& m = cfg.sensors[i];
            float sx = pos.x + std::cos(theta) * m.x_m - std::sin(theta) * m.y_m;
            float sy = pos.y + std::sin(theta) * m.x_m + std::cos(theta) * m.y_m;
            float ang = theta + (m.angle_deg + bot.sensor_angle_bias_deg[i]) * (PI_F / 180.0f);
            bool hit = bot.dists[i] < cfg.tof_max_range_m - 0.01f;
            ticks << "," << (hit ? Num(sx + std::cos(ang) * bot.dists[i]) : "null") << ","
                  << (hit ? Num(sy + std::sin(ang) * bot.dists[i]) : "null");
        }
        // Encoder column = what the controller saw this tick (Robot::enc_l/enc_r: the motion of the
        // tick before, as the encoder model reports it).
        ticks << "," << Num(bot.enc_l) << "," << Num(bot.enc_r)
              << "," << Num(bot.last_eff_v_l) << "," << Num(bot.last_eff_v_r) << "," << Num(head_err) << ","
              << Num(centre_err) << "," << Num(clearance);
        WriteLines(ticks, bot.brain, pos.x, pos.y, theta);
        WriteFitPoints(ticks, bot.brain, pos.x, pos.y, theta);
        ticks << "]";

        if (!bot.alive) { outcome = "CRASHED"; break; }
        if (bot.finished) { outcome = "FINISHED"; break; }
    }

    // ------------------------------------------------------------------------------------------ output
    std::string controller = ControllerName();
    std::time_t now = std::time(nullptr);
    char stamp[32];
    std::strftime(stamp, sizeof stamp, "%Y-%m-%d_%H-%M-%S", std::localtime(&now));
    if (a.out.empty()) {
        fs::create_directories("videos");
        a.out = "videos/" + a.track + "_" + stamp + ".html";
    }

    std::ostringstream data;
    data << "{\"meta\":{\"track\":\"" << a.track << "\",\"model\":\"" << a.model << "\",\"controller\":\""
         << controller << "\",\"seed\":" << a.seed << ",\"outcome\":\"" << outcome << "\",\"steps\":" << steps
         << ",\"dt\":" << Num(SIMULATION_DT) << ",\"recorded\":\"" << stamp << "\",\"has_lines\":" << (HasLines<Brain>::value ? "true" : "false") << "},";
    data << "\"robot\":{\"chassis_length\":" << Num(cfg.chassis_length_m) << ",\"chassis_width\":" << Num(cfg.chassis_width_m)
         << ",\"track_width\":" << Num(cfg.track_width_m) << ",\"wheel_length\":" << Num(cfg.wheel_length_m)
         << ",\"wheel_width\":" << Num(cfg.wheel_width_m) << ",\"wheel_offset\":" << Num(cfg.wheel_offset_from_center_m)
         << ",\"chassis_front\":" << Num(Robot::ChassisFront(cfg)) << ",\"chassis_rear\":" << Num(Robot::ChassisRear(cfg))
         << ",\"caster_x\":" << Num(cfg.caster_x_m) << ",\"caster_d\":" << Num(cfg.caster_diameter_m)
         << ",\"sensor_pos\":[[" << Num(cfg.sensors[0].x_m) << "," << Num(cfg.sensors[0].y_m) << "],[" << Num(cfg.sensors[1].x_m) << ","
         << Num(cfg.sensors[1].y_m) << "],[" << Num(cfg.sensors[2].x_m) << "," << Num(cfg.sensors[2].y_m) << "]]"
         << ",\"max_range\":" << Num(cfg.tof_max_range_m) << ",\"max_speed\":" << Num(cfg.max_linear_speed_mps) << ",\"sensor_angles\":[" << Num(cfg.sensors[0].angle_deg) << ","
         << Num(cfg.sensors[1].angle_deg) << "," << Num(cfg.sensors[2].angle_deg) << "]},";
    data << "\"walls\":[";
    for (size_t i = 0; i < t.walls.size(); ++i) {
        const Wall& w = t.walls[i];
        data << (i ? "," : "") << "[" << Num(w.p1.x) << "," << Num(w.p1.y) << "," << Num(w.p2.x) << "," << Num(w.p2.y) << "]";
    }
    data << "],\"finish\":";
    if (finish) data << "[" << Num(finish->p1.x) << "," << Num(finish->p1.y) << "," << Num(finish->p2.x) << "," << Num(finish->p2.y) << "]";
    else data << "null";
    if (!resims.empty()) {
        data << ",\"orig_top\":" << Num(v0) << ",\"orig_s\":[";
        for (size_t i = 0; i < orig.size(); ++i) data << (i ? "," : "") << Num(orig[i].s);
        data << "],\"resims\":" << ResimJson(resims, orig);
    }
    data << ",\"ticks\":[" << ticks.str() << "]}";

    std::string html = kPlayerHtml;
    const std::string marker = "/*__REPLAY_DATA__*/null";
    size_t at = html.find(marker);
    html.replace(at, marker.size(), data.str());
    std::string title = a.track + " - " + controller + " - " + outcome;
    const std::string tmark = "__TITLE__";
    for (size_t p; (p = html.find(tmark)) != std::string::npos;) html.replace(p, tmark.size(), title);

    std::ofstream f(a.out, std::ios::binary);
    f << html;
    f.close();
    std::cout << "Recorded " << steps << " steps (" << outcome << ") on " << a.track << " -> " << a.out << "\n";
    if (a.open) {
#ifdef _WIN32
        std::string cmd = "start \"\" \"" + fs::absolute(a.out).string() + "\"";
#elif defined(__APPLE__)
        std::string cmd = "open \"" + a.out + "\"";
#else
        std::string cmd = "xdg-open \"" + a.out + "\" >/dev/null 2>&1 &";
#endif
        if (std::system(cmd.c_str()) != 0) std::cout << "Open it in a browser: " << a.out << "\n";
    }
    return 0;
}

#include "replay_player.inc"
