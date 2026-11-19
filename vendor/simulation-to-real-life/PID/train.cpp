#include "Types.hpp"
#include "Raycast.hpp"
#include "PidController.hpp"
#include "Track.hpp"
#include "Robot.hpp"
#include "Spawn.hpp"
#include "SimConfig.hpp"
#include "GraphExportHelper.hpp"
#include <iostream>
#include <vector>
#include <random>
#include <algorithm>
#include <chrono>
#include <csignal>
#include <atomic>
#include "xlsxwriter.h"

struct Candidate {
    PidBrain brain;
    float fitness = 0.0f;
};

// Episodes run per track per generation. Crash-freeness measured over a single episode is a coin
// flip, not a property of the brain, so the curriculum and the checkpoint rule both need several.
// Cost is linear: EPISODES_PER_TRACK x the simulation time per generation.
static const int EPISODES_PER_TRACK = 4;

enum class EpisodeOutcome { Crashed, Finished, OutOfSteps };

struct EpisodeResult {
    EpisodeOutcome outcome = EpisodeOutcome::OutOfSteps;
    int steps_used = 0;
};

// One track's EPISODES_PER_TRACK runs, collapsed: mean score plus how the episodes ended.
struct TrackResult {
    float score = 0.0f;
    int crashes = 0;
    int finishes = 0;
    int mean_steps = 0;
    EpisodeOutcome outcome = EpisodeOutcome::OutOfSteps;
};

struct GenMetric {
    int gen;
    float best_fitness;
    float avg_fitness;
    long long elapsed_s;
};

static std::vector<GenMetric> g_training_metrics;
static std::string g_export_xlsx_path;
static std::atomic<bool> g_stop_requested(false);

// Run seed. Mixed into every random stream in the run (population init, mutation, per-episode
// sensor/encoder noise and wall perturbation), so that two runs with different seeds are genuinely
// independent samples of the same training procedure, and two runs with the SAME seed under
// different sensor models differ only by the sensor model. Without this the trainer is fully
// deterministic and every "repeat" returns the identical brain.
static unsigned int g_run_seed = 0;

// Cheap integer mix, so nearby seeds do not produce correlated streams.
static inline unsigned int SeedMix(unsigned int a, unsigned int b) {
    unsigned int h = a * 0x9E3779B9u ^ (b + 0x85EBCA6Bu + (a << 6) + (a >> 2));
    h ^= h >> 15; h *= 0x2C1B3C6Du; h ^= h >> 12;
    return h;
}

void SignalHandler(int signum) {
    (void)signum;
    g_stop_requested = true;
}

// episode: which of the EPISODES_PER_TRACK runs this is. It varies both random streams, so the
// same brain meets a different wall perturbation AND a different sensor/encoder noise sequence in
// each run. One episode only ever samples one draw of a stochastic outcome; several sample the
// distribution, which is what makes "does this brain crash here" answerable at all.
//
// STRAIGHT-TRACK REWARD. The goal is: start badly rotated (up to ~80 deg) and possibly off-centre,
// then drive the whole straight PARALLEL to the track and CENTRED between the walls, ignoring wall
// errors. Every metre gained along the track axis is worth
//     PROGRESS_WEIGHT * (1 + parallel_weight * p + center_weight * c)
// where p, c in [0, 1] are Gaussian scores of the heading error and the centre error, measured
// against the track's ground-truth guide (global axis + median-filtered centreline), so a slanted
// wall segment or a bump does not move the target. Tying the bonus to distance, not time, means
// sitting still centred earns nothing and recovering alignment quickly earns the most.
float EvaluateRobot(PidBrain& brain, const LoadedTrack& t, const SimConfig& cfg, int generation,
                    int episode = 0, EpisodeResult* result = nullptr) {
    std::mt19937 rng(SeedMix(g_run_seed, 42u + (unsigned)episode * 7919u));
    std::mt19937 track_rng(SeedMix(g_run_seed ^ 0xABCDu,
                                   (unsigned)(generation * EPISODES_PER_TRACK + episode)));
    std::vector<Wall> active_track = PerturbTrack(t.walls, cfg.wall_noise_std_m, track_rng);
    const StraightGuide& guide = t.guide;

    // Spawn: episode 0 is the track file's spawn; the others draw a random pose (Spawn.hpp): random lateral
    // position, heading offset up to the largest the corridor width allows, never parallel, never in a wall.
    Vec2 spawn_axle = Robot::SpawnAxle(cfg, t.spawn, t.spawn_heading);
    float spawn_heading = t.spawn_heading;
    if (episode > 0) RandomSpawn(t, cfg, active_track, track_rng, spawn_axle, spawn_heading);

    Robot bot;
    bot.brain = brain;
    bot.Reset(spawn_axle, spawn_heading, cfg.tof_max_range_m);
    bot.SampleHardwareBias(cfg, rng);  // a different "build" of the robot per episode

    float progress_score = 0.0f;         // metres along the axis, each weighted by parallel/centre quality
    float forward_m = 0.0f;              // plain metres along the axis (for the average speed of a finished run)
    float backward_m = 0.0f;             // metres lost going backwards along the axis
    float action_change_penalty = 0.0f;  // sum of squared tick-to-tick change in normalized wheel commands
    float approach_penalty = 0.0f;       // sum of squared closing speed beyond the noise deadband
    float wobble_penalty = 0.0f;         // turn-direction reversals, weighted by how hard the new turn is
    int last_turn_sign = 0;              // direction of the last significant turn: +1 left, -1 right
    bool died = false;
    bool finished = false;
    int steps_used = 0;
    const Wall* finish_seg = t.finish.has_finish ? &t.finish.segment : nullptr;
    float prev_a_l = 0.0f;
    float prev_a_r = 0.0f;
    float prev_min_dist = 0.0f;

    const int MAX_STEPS = cfg.MaxSteps();
    const float PROGRESS_WEIGHT = 100.0f;
    const float CHASSIS_HALF_W = 0.5f * cfg.chassis_width_m;
    const float heading_sigma = cfg.heading_sigma_deg * (PI_F / 180.0f);

    float prev_s = guide.At(bot.pos, bot.theta).s;

    for (int step = 0; step < MAX_STEPS; ++step) {
        bot.Step(active_track, cfg, rng, finish_seg);
        steps_used = step + 1;

        if (!bot.alive) {
            died = true;
            break;
        }

        // TERM 1: quality-weighted progress along the track axis (ground-truth pose, not wheel
        // odometry, so spinning or weaving in place earns nothing).
        StraightGuide::Query q = guide.At(bot.pos, bot.theta);
        if (q.ok) {
            float ds = q.s - prev_s;
            prev_s = q.s;
            if (ds > 0.0f) {
                forward_m += ds;
                float p = std::exp(-(q.heading_err * q.heading_err) / (heading_sigma * heading_sigma));
                float free_half = std::max(0.02f, q.half_width - CHASSIS_HALF_W);
                float e = q.center_err / (cfg.center_sigma_frac * free_half);
                float c = std::exp(-e * e);
                progress_score += ds * (1.0f + cfg.parallel_weight * p + cfg.center_weight * c);
            } else {
                backward_m -= ds;
            }
        }

        // TERM 2: quadratic action-change penalty (squared, not abs, to avoid bang-off-bang).
        float a_l = bot.v_l / bot.brain.hw.max_linear_speed;
        float a_r = bot.v_r / bot.brain.hw.max_linear_speed;
        if (step > 0) {
            float d_l = a_l - prev_a_l;
            float d_r = a_r - prev_a_r;
            // Time-consistent: (change per second)^2 integrated over time, scaled so a 50 ms tick gives the
            // historical value. A jump costs more at a faster rate, a smooth ramp costs the same.
            action_change_penalty += (d_l * d_l + d_r * d_r) * (0.05f / SIMULATION_DT);
        }
        prev_a_l = a_l;
        prev_a_r = a_r;

        // TERM 3: penalize closing on the nearest wall faster than ToF noise can explain.
        float min_dist = std::min({bot.dists[0], bot.dists[1], bot.dists[2]});
        if (step > 0) {
            float d_min = (min_dist - prev_min_dist) / SIMULATION_DT;  // m/s
            // The deadband is the ToF noise floor of a one-tick difference, which grows as 1/dt.
            const float deadband = cfg.approach_deadband_mps * (0.05f / SIMULATION_DT);
            if (d_min < -deadband) {
                float closing = std::min(-d_min - deadband, 1.0f);
                approach_penalty += closing * closing * (SIMULATION_DT / 0.05f);  // per 50 ms of closing
            }
        }
        prev_min_dist = min_dist;

        // TERM 5: wobble. Rotating left, then right, then left again is penalized at every
        // reversal, scaled by the new turn rate (capped), so one big swing costs more than a tiny
        // correction. Turns below the deadband are ignored, so the small steady corrections needed
        // to hold the centre are free, and so is one long turn to recover the starting rotation.
        // The scripted scan (left then right on purpose) is not the network's doing and is skipped.
        if (bot.state == RobotState::NAVIGATING) {
            float omega = (bot.last_eff_v_r - bot.last_eff_v_l) / bot.brain.hw.track_width;  // rad/s
            if (std::abs(omega) > cfg.wobble_omega_deadband) {
                int sign = omega > 0.0f ? 1 : -1;
                if (last_turn_sign != 0 && sign != last_turn_sign) {
                    wobble_penalty += std::min(std::abs(omega), 3.0f);
                }
                last_turn_sign = sign;
            }
        } else {
            last_turn_sign = 0;
        }

        if (bot.finished) {
            finished = true;
            break;
        }
    }

    // TERM 4: crash. Backward metres cost as much as a perfect forward metre earns.
    float score = progress_score * PROGRESS_WEIGHT
                - backward_m * PROGRESS_WEIGHT * (1.0f + cfg.parallel_weight + cfg.center_weight)
                - action_change_penalty * cfg.smooth_weight
                - approach_penalty * 2.0f
                - wobble_penalty * cfg.wobble_weight
                - (died ? cfg.crash_penalty : 0.0f);
    if (finished) {
        // Credit the score forfeited by ending early, at the episode's average rate, so finishing
        // always beats running out the clock at the same quality...
        float rate = progress_score * PROGRESS_WEIGHT / (float)steps_used;
        if (rate > 0.0f) score += rate * (float)(MAX_STEPS - steps_used);
        // ...plus a bonus scaled by time saved, so finishing fast beats finishing slow.
        score += cfg.finish_bonus * (1.0f - (float)steps_used / (float)MAX_STEPS);
        // ...plus a SPEED reward: average speed along the track / top speed. The finish bonus above is scaled by
        // the share of the whole episode limit (240 s) left over, which hardly separates a 17 s from a 20 s run.
        float avg_v = forward_m / std::max(1e-3f, (float)steps_used * SIMULATION_DT);
        score += cfg.speed_bonus * std::clamp(avg_v / bot.brain.hw.max_linear_speed, 0.0f, 1.0f);
    }

    if (result) {
        result->outcome = died ? EpisodeOutcome::Crashed : finished ? EpisodeOutcome::Finished : EpisodeOutcome::OutOfSteps;
        result->steps_used = steps_used;
    }
    return score;
}

// Weighted mean score across the training tracks. Weights come from the curriculum below:
// a track the champion crashes on is worth more, so selection pressure moves to where it fails.
// With all weights at 1 this is the plain mean. Optionally reports the per-track episodes.
float EvaluateOnTracks(PidBrain& brain, const std::vector<LoadedTrack>& tracks, const std::vector<float>& weights,
                       const SimConfig& cfg, int generation, std::vector<TrackResult>* out_results = nullptr) {
    float sum = 0.0f;
    float weight_sum = 0.0f;
    for (size_t i = 0; i < tracks.size(); ++i) {
        TrackResult tr;
        float track_sum = 0.0f;
        int steps_sum = 0;
        for (int e = 0; e < EPISODES_PER_TRACK; ++e) {
            EpisodeResult r;
            track_sum += EvaluateRobot(brain, tracks[i], cfg, generation, e, &r);
            steps_sum += r.steps_used;
            if (r.outcome == EpisodeOutcome::Crashed) ++tr.crashes;
            else if (r.outcome == EpisodeOutcome::Finished) ++tr.finishes;
        }
        // A track counts as crashed if it crashed even once: the goal is a brain that never
        // crashes, so one crash in four is a failure, not a 75% success.
        tr.score = track_sum / (float)EPISODES_PER_TRACK;
        tr.mean_steps = steps_sum / EPISODES_PER_TRACK;
        tr.outcome = tr.crashes > 0 ? EpisodeOutcome::Crashed
                   : tr.finishes == EPISODES_PER_TRACK ? EpisodeOutcome::Finished
                   : EpisodeOutcome::OutOfSteps;

        sum += tr.score * weights[i];
        weight_sum += weights[i];
        if (out_results) (*out_results)[i] = tr;
    }
    return sum / weight_sum;
}

// Curriculum: crashing on a track raises its weight, finishing decays it back toward 1, and
// running out of steps holds it. Once nothing crashes, every weight relaxes to 1 and training
// is uniform again -- "fix the failures first, then maximize everything together".
struct Curriculum {
    static constexpr float MAX_WEIGHT = 8.0f;
    static constexpr float CRASH_GROWTH = 1.6f;
    static constexpr float FINISH_DECAY = 0.90f;

    std::vector<float> weights;

    explicit Curriculum(size_t n) : weights(n, 1.0f) {}

    void Update(const std::vector<TrackResult>& results) {
        for (size_t i = 0; i < results.size(); ++i) {
            switch (results[i].outcome) {
                case EpisodeOutcome::Crashed:
                    weights[i] = std::min(weights[i] * CRASH_GROWTH, MAX_WEIGHT);
                    break;
                case EpisodeOutcome::Finished:
                    weights[i] = std::max(weights[i] * FINISH_DECAY, 1.0f);
                    break;
                case EpisodeOutcome::OutOfSteps:
                    break;  // partial credit: neither punish nor forgive
            }
        }
    }

    bool IsUniform() const {
        for (float w : weights) {
            if (w > 1.0001f) return false;
        }
        return true;
    }
};


void WriteTrainingXlsx(const std::string& filepath, const std::vector<GenMetric>& metrics) {
    if (metrics.empty()) return;

    lxw_workbook* workbook = workbook_new(filepath.c_str());
    if (!workbook) {
        std::cerr << "[GraphExport] Error: Failed to create workbook at " << filepath << "\n";
        return;
    }

    lxw_worksheet* worksheet = workbook_add_worksheet(workbook, "Convergence");
    
    lxw_format* header_fmt = workbook_add_format(workbook);
    format_set_bold(header_fmt);

    worksheet_write_string(worksheet, 0, 0, "Generation", header_fmt);
    worksheet_write_string(worksheet, 0, 1, "Best Fitness", header_fmt);
    worksheet_write_string(worksheet, 0, 2, "Avg Fitness", header_fmt);
    worksheet_write_string(worksheet, 0, 3, "Elapsed (s)", header_fmt);

    for (size_t i = 0; i < metrics.size(); ++i) {
        lxw_row_t row = (lxw_row_t)(i + 1);
        worksheet_write_number(worksheet, row, 0, (double)metrics[i].gen, NULL);
        worksheet_write_number(worksheet, row, 1, (double)metrics[i].best_fitness, NULL);
        worksheet_write_number(worksheet, row, 2, (double)metrics[i].avg_fitness, NULL);
        worksheet_write_number(worksheet, row, 3, (double)metrics[i].elapsed_s, NULL);
    }

    lxw_chart* chart = workbook_add_chart(workbook, LXW_CHART_LINE);
    std::string last_row_str = std::to_string(metrics.size() + 1);

    std::string cat_range = "=Convergence!$A$2:$A$" + last_row_str;
    std::string val_range_best = "=Convergence!$B$2:$B$" + last_row_str;
    std::string val_range_avg = "=Convergence!$C$2:$C$" + last_row_str;

    lxw_chart_series* series1 = chart_add_series(chart, cat_range.c_str(), val_range_best.c_str());
    chart_series_set_name(series1, "=Convergence!$B$1");

    lxw_chart_series* series2 = chart_add_series(chart, cat_range.c_str(), val_range_avg.c_str());
    chart_series_set_name(series2, "=Convergence!$C$1");

    chart_title_set_name(chart, "Training Fitness Convergence");
    chart_axis_set_name(chart->x_axis, "Generation");
    chart_axis_set_name(chart->y_axis, "Fitness Score");

    worksheet_insert_chart(worksheet, 1, 5, chart);

    workbook_close(workbook);
}

// Command line, all optional. With no arguments this behaves exactly as it always has: seed 0,
// resume from and checkpoint to best_pid_model.txt, run until Ctrl+C.
//
//   --seed N            run seed; mixed into every random stream (default 0)
//   --gens N            stop after N generations instead of running until Ctrl+C
//   --patience N        convergence stop: end once N generations pass without a new checkpoint
//   --seconds N         stop after N seconds of wall-clock training (checked between generations)
//   --out PATH          checkpoint file to write (default best_pid_model.txt)
//   --resume PATH|none  checkpoint to resume from; "none" starts from a random population
//   --signal-model 0|1  override error_params.txt tof_signal_model
//   --rays N            override error_params.txt tof_rays
//   --pso               particle swarm optimisation instead of the GA (same population, init, evaluation)
//   --curve FILE        convergence CSV: gen, simulator episodes so far, champion uniform fitness, its crashes
//   --quiet             suppress training_log.csv and the xlsx chart (for batch experiments,
//                       which would otherwise have every run append to the same log)
struct TrainArgs {
    unsigned int seed = 0;
    int gens = 0;                            // 0 = unbounded
    int patience = 0;                        // 0 = never stop for lack of improvement
    int seconds = 0;                         // 0 = unbounded
    std::string out = "best_pid_model.txt";
    std::string resume = "best_pid_model.txt";
    int signal_model = -1;                   // -1 = take it from error_params.txt
    int rays = -1;
    bool quiet = false;
    bool pso = false;                        // JOB 5: particle swarm instead of the GA (tools/exp/pso.hpp)
    std::string curve;                       // CSV: gen,episodes,uniform_fitness,crashes,saved_crashes per generation
};

static TrainArgs ParseArgs(int argc, char** argv) {
    TrainArgs a;
    for (int i = 1; i < argc; ++i) {
        std::string k = argv[i];
        auto next = [&]() -> std::string { return (i + 1 < argc) ? argv[++i] : std::string(); };
        if (k == "--seed") a.seed = (unsigned int)std::stoul(next());
        else if (k == "--gens") a.gens = std::stoi(next());
        else if (k == "--patience") a.patience = std::stoi(next());
        else if (k == "--seconds") a.seconds = std::stoi(next());
        else if (k == "--out") a.out = next();
        else if (k == "--resume") a.resume = next();
        else if (k == "--signal-model") a.signal_model = std::stoi(next());
        else if (k == "--rays") a.rays = std::stoi(next());
        else if (k == "--quiet") a.quiet = true;
        else if (k == "--pso") a.pso = true;
        else if (k == "--curve") a.curve = next();
        else std::cerr << "Warning: unknown argument '" << k << "' ignored.\n";
    }
    return a;
}

#include "../tools/exp/pso.hpp"

int main(int argc, char** argv) {
    std::signal(SIGINT, SignalHandler);

    TrainArgs args = ParseArgs(argc, argv);
    g_run_seed = args.seed;

    SimConfig config = LoadConfig("sim_config.txt");
    if (args.signal_model >= 0) config.tof_signal_model = (args.signal_model != 0);
    if (args.rays > 0) config.tof_rays = args.rays;
    std::cout << "Run seed " << args.seed
              << " | sensor model: " << (config.tof_signal_model ? "SIGNAL" : "min()")
              << " | rays: " << config.tof_rays
              << " | checkpoint: " << args.out;
    if (args.gens > 0) std::cout << " | stopping after " << args.gens << " generations";
    if (args.patience > 0) std::cout << " | converged after " << args.patience << " generations without a new best";
    if (args.seconds > 0) std::cout << " | stopping after " << args.seconds << " s";
    std::cout << "\n";
    const std::vector<LoadedTrack> tracks = LoadTrackSet(config.train_tracks);
    std::cout << "Training on " << tracks.size() << " track(s), " << EPISODES_PER_TRACK
              << " episodes each, score = weighted mean across tracks:\n";
    for (const auto& t : tracks) {
        std::cout << "  " << t.name << ": " << t.walls.size() << " walls, "
                  << (t.finish.has_finish ? "finish line" : "NO finish line (ends on crash or MAX_STEPS)");
        if (t.guide.valid) {
            StraightGuide::Query q = t.guide.At(t.spawn, t.spawn_heading);
            std::cout << ", axis " << t.guide.axis_angle * 180.0f / PI_F << " deg, spawn heading err "
                      << q.heading_err * 180.0f / PI_F << " deg, spawn centre err " << q.center_err
                      << " m, half width " << q.half_width << " m";
        } else {
            std::cout << ", NO straight guide (progress reward disabled)";
        }
        std::cout << "\n";
    }

    std::string export_folder = "Graphs/training_graphs/generation_to_fitness";
    if (!args.quiet) {
        g_export_xlsx_path = GraphExport::FormTrackXlsxPath(export_folder, TrackSetLabel(config.train_tracks));
        std::cout << "[GraphExport] Initialized training workbook: " << g_export_xlsx_path << "\n";
    }

    std::cout << "=================================================================\n";
    std::cout << "STRAIGHT WALL TRAINER (PID GAINS) - PARALLEL + CENTRED REWARD\n";
    std::cout << "=================================================================\n";

    const int POP_SIZE = 64;
    std::vector<Candidate> pop(POP_SIZE);

    PidBrain base_brain(config);
    bool resumed = (args.resume != "none") && base_brain.Load(args.resume);

    if (resumed) {
        std::cout << "Resumed from checkpoint: " << args.resume << "\n";
        pop[0].brain = base_brain;
        #pragma omp parallel for
        for (int i = 1; i < POP_SIZE; ++i) {
            std::mt19937 init_rng(SeedMix(g_run_seed, 1000u + (unsigned)i));
            pop[i].brain = base_brain;
            pop[i].brain.Mutate(init_rng);
        }
    } else {
        std::cout << "No checkpoint found. Initializing random population.\n";
        for (int i = 0; i < POP_SIZE; ++i) {
            pop[i].brain = PidBrain(config, SeedMix(g_run_seed, 1000u + (unsigned)i));
        }
    }

    auto start_time = std::chrono::steady_clock::now();
    int gen = 0;

    Candidate champion = pop[0];
    Curriculum curriculum(tracks.size());

    // Checkpointing is judged on the UNIFORM mean, never the curriculum-weighted score, so that
    // shifting weights cannot make a worse network look like an improvement. Fewer crashes wins
    // first; the uniform mean only breaks ties.
    int saved_crashes = (int)tracks.size() * EPISODES_PER_TRACK + 1;
    int last_save_gen = 0;  // for --patience
    float saved_uniform = -1e9f;

    std::vector<TrackResult> track_results(tracks.size());
    const std::vector<float> uniform_weights(tracks.size(), 1.0f);

    pso::State swarm;
    if (args.pso) swarm.Init(pop);
    std::ofstream curve_f;
    if (!args.curve.empty()) { curve_f.open(args.curve); curve_f << "gen,episodes,uniform_fitness,crashes,saved_crashes\n"; }
    long long episodes = 0;

    auto time_left = [&]() {
        if (args.seconds <= 0) return true;
        return std::chrono::steady_clock::now() - start_time < std::chrono::seconds(args.seconds);
    };
    while (!g_stop_requested && (args.gens <= 0 || gen < args.gens) && time_left()) {
        gen++;

        // pop[0] is last generation's champion and is re-evaluated here alongside its own mutants,
        // on this generation's wall perturbation. Comparing a stale score from an older, easier
        // perturbation against fresh ones is what froze the old champion in place.
        #pragma omp parallel for
        for (int i = 0; i < POP_SIZE; ++i) {
            pop[i].fitness = EvaluateOnTracks(pop[i].brain, tracks, curriculum.weights, config, gen);
        }

        episodes += (long long)POP_SIZE * (long long)tracks.size() * EPISODES_PER_TRACK;
        if (args.pso) swarm.Observe(pop);  // particle order, before the sort
        std::sort(pop.begin(), pop.end(), [](const Candidate& a, const Candidate& b) {
            return a.fitness > b.fitness;
        });

        champion = pop[0];

        // Per-track breakdown of this generation's champion: drives both the curriculum and the
        // checkpoint decision.
        float uniform_fitness = EvaluateOnTracks(champion.brain, tracks, uniform_weights, config, gen,
                                                 &track_results);
        int crashes = 0;
        for (const auto& r : track_results) {
            crashes += r.crashes;  // total crashed episodes, so 1-in-4 beats 4-in-4
        }
        episodes += (long long)tracks.size() * EPISODES_PER_TRACK;

        if (crashes < saved_crashes || (crashes == saved_crashes && uniform_fitness > saved_uniform)) {
            saved_crashes = crashes;
            saved_uniform = uniform_fitness;
            champion.brain.Save(args.out);
            last_save_gen = gen;
        }

        curriculum.Update(track_results);
        if (curve_f.is_open())
            curve_f << gen << "," << episodes << "," << uniform_fitness << "," << crashes << ","
                    << std::min(saved_crashes, crashes) << std::endl;

        float avg_fitness = 0.0f;
        for (const auto& cand : pop) {
            avg_fitness += cand.fitness;
        }
        avg_fitness /= POP_SIZE;

        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - start_time).count();

        if (!args.quiet) {
            bool exists = false;
            {
                std::ifstream check("training_log.csv");
                if (check.good()) exists = true;
            }
            std::ofstream log_file("training_log.csv", std::ios::app);
            if (!exists) {
                log_file << "gen,best_fitness,avg_fitness,elapsed_s\n";
            }
            log_file << gen << "," << uniform_fitness << "," << avg_fitness << "," << elapsed << "\n";
        }

        g_training_metrics.push_back({gen, uniform_fitness, avg_fitness, elapsed});

        if (gen % 25 == 0) {
            std::cout << "[Gen " << gen << " | " << elapsed << "s] Champion (uniform): " << uniform_fitness
                      << " | Weighted: " << champion.fitness
                      << " | Saved: " << saved_uniform << " (" << saved_crashes << " crashed episodes)"
                      << " | Phase: " << (curriculum.IsUniform() ? "UNIFORM" : "FOCUS") << "\n";
            std::cout << "    PID gains: Kp " << champion.brain.kp << "  Ki " << champion.brain.ki
                      << "  Kd " << champion.brain.kd << "  cruise " << champion.brain.base_speed_frac * 100.0f << "%\n";
            for (size_t i = 0; i < tracks.size(); ++i) {
                std::cout << "    " << tracks[i].name << ": " << track_results[i].score
                          << " (" << track_results[i].finishes << "/" << EPISODES_PER_TRACK << " finished, "
                          << track_results[i].crashes << " crashed, " << track_results[i].mean_steps
                          << " steps avg, weight x" << curriculum.weights[i] << ")\n";
            }
            if (!args.quiet) WriteTrainingXlsx(g_export_xlsx_path, g_training_metrics);
        }

        if (args.patience > 0 && gen - last_save_gen >= args.patience) {
            std::cout << "[Converged] no new best for " << args.patience << " generations (last at gen "
                      << last_save_gen << ").\n";
            break;
        }

        // Elitism within the generation: the winner seeds the next population unmutated at index 0,
        // so it is always re-tested rather than assumed still best.
        if (args.pso) {  // the swarm moves; pop is rewritten in particle order
            std::mt19937 pso_rng(SeedMix(g_run_seed, (unsigned)(gen * 100 + 7)));
            swarm.Step(pop, pso_rng);
            continue;
        }
        pop[0] = champion;
        for (int i = 1; i < POP_SIZE; ++i) {
            std::mt19937 mutate_rng(SeedMix(g_run_seed, (unsigned)(gen * 100 + i)));
            pop[i].brain = champion.brain;
            pop[i].brain.Mutate(mutate_rng);
        }
    }

    if (!args.quiet) {
        std::cout << "\n[GraphExport] Finalizing Excel output on shutdown...\n";
        WriteTrainingXlsx(g_export_xlsx_path, g_training_metrics);
        std::cout << "[GraphExport] Saved " << g_training_metrics.size() << " generations to " << g_export_xlsx_path << "\n";
    }
    {
        PidBrain saved(config);
        if (saved.Load(args.out)) {
            std::cout << "[Done] saved PID gains: Kp " << saved.kp << "  Ki " << saved.ki << "  Kd " << saved.kd
                      << "  cruise " << saved.base_speed_frac * 100.0f << "%\n";
        }
    }
    std::cout << "[Done] seed " << args.seed << ", " << gen << " generations, best -> " << args.out
              << " (" << saved_crashes << " crashed episodes, uniform fitness " << saved_uniform << ")\n";

    return 0;
}