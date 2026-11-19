#include "Types.hpp"
#include "Raycast.hpp"
#include "PidController.hpp"
#include "Track.hpp"
#include "Robot.hpp"
#include "SimConfig.hpp"
#include "GraphExportHelper.hpp"
#include <iostream>
#include <vector>
#include <random>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include "xlsxwriter.h"

struct BenchmarkRow {
    double multiplier;
    int crashes;
    double crash_rate_pct;
    int finishes;
    double completion_rate_pct;
    double avg_completion_time_s;  // mean over finishers only
    bool has_finisher;
};

void ExportBenchmarkXlsx(const std::string& track_name, const std::vector<BenchmarkRow>& rows) {
    std::string folder = "Graphs/benchmark_graphs/noise_to_crash_rate";
    std::string filepath = GraphExport::FormTrackXlsxPath(folder, track_name);

    lxw_workbook* wb = workbook_new(filepath.c_str());
    if (!wb) return;
    lxw_worksheet* ws = workbook_add_worksheet(wb, "NoiseBenchmark");

    lxw_format* header_fmt = workbook_add_format(wb);
    format_set_bold(header_fmt);

    worksheet_write_string(ws, 0, 0, "Noise_Multiplier", header_fmt);
    worksheet_write_string(ws, 0, 1, "Crash_Rate_Pct", header_fmt);
    worksheet_write_string(ws, 0, 2, "Avg_Completion_Time_s", header_fmt);
    worksheet_write_string(ws, 0, 3, "Completion_Rate_Pct", header_fmt);

    for (size_t i = 0; i < rows.size(); ++i) {
        lxw_row_t r = (lxw_row_t)(i + 1);
        worksheet_write_number(ws, r, 0, rows[i].multiplier, NULL);
        worksheet_write_number(ws, r, 1, rows[i].crash_rate_pct, NULL);
        if (rows[i].has_finisher) {
            worksheet_write_number(ws, r, 2, rows[i].avg_completion_time_s, NULL);
        } else {
            worksheet_write_string(ws, r, 2, "N/A", NULL);
        }
        worksheet_write_number(ws, r, 3, rows[i].completion_rate_pct, NULL);
    }

    std::string num_rows_str = std::to_string(rows.size() + 1);

    lxw_chart* chart1 = workbook_add_chart(wb, LXW_CHART_LINE);
    std::string cat_range1 = "=NoiseBenchmark!$A$2:$A$" + num_rows_str;
    std::string val1_range = "=NoiseBenchmark!$B$2:$B$" + num_rows_str;
    lxw_chart_series* s1 = chart_add_series(chart1, cat_range1.c_str(), val1_range.c_str());
    chart_series_set_name(s1, "=NoiseBenchmark!$B$1");
    chart_title_set_name(chart1, "Sim-to-Real Crash Rate vs Noise Multiplier");
    chart_axis_set_name(chart1->x_axis, "Noise Multiplier (x)");
    chart_axis_set_name(chart1->y_axis, "Crash Rate (%)");
    worksheet_insert_chart(ws, 1, 4, chart1);

    lxw_chart* chart2 = workbook_add_chart(wb, LXW_CHART_LINE);
    std::string val2_range = "=NoiseBenchmark!$C$2:$C$" + num_rows_str;
    lxw_chart_series* s2 = chart_add_series(chart2, cat_range1.c_str(), val2_range.c_str());
    chart_series_set_name(s2, "=NoiseBenchmark!$C$1");
    chart_title_set_name(chart2, "Mean Completion Time (Finishers) vs Noise Multiplier");
    chart_axis_set_name(chart2->x_axis, "Noise Multiplier (x)");
    chart_axis_set_name(chart2->y_axis, "Completion Time (s)");
    worksheet_insert_chart(ws, 17, 4, chart2);

    workbook_close(wb);
    std::cout << "[GraphExport] Saved benchmark workbook: " << filepath << "\n";
}

struct TierCounts {
    int crashes = 0;
    int finishes = 0;
    int trials = 0;
    double total_finish_time = 0.0;
};

static BenchmarkRow MakeRow(double multiplier, const TierCounts& c) {
    BenchmarkRow row;
    row.multiplier = multiplier;
    row.crashes = c.crashes;
    row.crash_rate_pct = c.trials ? 100.0 * c.crashes / c.trials : 0.0;
    row.finishes = c.finishes;
    row.completion_rate_pct = c.trials ? 100.0 * c.finishes / c.trials : 0.0;
    row.has_finisher = c.finishes > 0;
    row.avg_completion_time_s = c.finishes ? c.total_finish_time / c.finishes : 0.0;
    return row;
}

static void PrintRow(std::ostream& out, const std::string& label, const BenchmarkRow& row) {
    out << "  " << std::left << std::setw(20) << label << std::right
        << std::setw(5) << std::fixed << std::setprecision(1) << row.multiplier << "x | "
        << std::setw(7) << row.crashes << " | "
        << std::setw(9) << std::setprecision(1) << row.crash_rate_pct << "% | "
        << std::setw(8) << row.finishes << " | "
        << std::setw(9) << std::setprecision(1) << row.completion_rate_pct << "% | ";
    if (row.has_finisher) out << std::setw(10) << std::setprecision(2) << row.avg_completion_time_s << " s\n";
    else out << std::setw(10) << "N/A (no finishers)\n";
}

// Usage: benchmark [--heldout] [--tracks a,b,c] [--trials N]
//   --heldout  benchmark the held-out tracks (sim_config.txt heldout_tracks), never used in training
//   --tracks   an explicit comma-separated track list
int main(int argc, char** argv) {
    SimConfig base_cfg = LoadConfig("sim_config.txt");
    std::vector<std::string> track_names = base_cfg.train_tracks;
    int trials_arg = 0;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--heldout") track_names = base_cfg.heldout_tracks;
        else if (a == "--tracks" && i + 1 < argc) track_names = SplitTrackList(argv[++i]);
        else if (a == "--trials" && i + 1 < argc) trials_arg = std::atoi(argv[++i]);
    }
    if (track_names.empty()) { std::cerr << "No tracks (heldout_tracks missing from sim_config.txt?)\n"; return 2; }
    const std::vector<LoadedTrack> tracks = LoadTrackSet(track_names);

    PidBrain brain(base_cfg);
    if (!brain.Load("best_pid_model.txt")) {
        std::cerr << "Warning: Could not load best_pid_model.txt, evaluating default (untrained) PID gains.\n";
    }

    const float multipliers[] = {0.0f, 0.5f, 1.0f, 2.0f};
    const int NUM_MULT = 4;
    const int NUM_TRIALS = trials_arg > 0 ? trials_arg : 50;
    const int MAX_STEPS = base_cfg.MaxSteps();

    std::cout << "=======================================================================================\n";
    std::cout << " SIM-TO-REAL SENSITIVITY BENCHMARK (" << NUM_TRIALS << " Trials per Tier, per Track)\n";
    std::cout << " Tracks: ";
    for (size_t i = 0; i < tracks.size(); ++i) std::cout << (i ? ", " : "") << tracks[i].name;
    std::cout << "\n";
    for (const auto& t : tracks) {
        if (!t.finish.has_finish) std::cout << " (" << t.name << " has no finish line: completion is 0% by definition)\n";
    }
    std::cout << "=======================================================================================\n";
    std::cout << "  Track                Mult | Crashes | Crash Rate | Finished | Completion | Mean Time (Finishers)\n";
    std::cout << "---------------------------------------------------------------------------------------\n";

    std::ofstream csv("benchmark_results.csv");
    csv << "track,multiplier,crash_rate_pct,completion_rate_pct,avg_finish_time_s\n";
    auto csv_row = [&](const std::string& name, const BenchmarkRow& r) {
        csv << name << "," << r.multiplier << "," << r.crash_rate_pct << "," << r.completion_rate_pct << ",";
        if (r.has_finisher) csv << r.avg_completion_time_s << "\n"; else csv << "N/A\n";
    };

    TierCounts overall[NUM_MULT];

    for (const auto& t : tracks) {
        const Wall* finish_seg = t.finish.has_finish ? &t.finish.segment : nullptr;
        std::vector<BenchmarkRow> rows;

        for (int m = 0; m < NUM_MULT; ++m) {
            float M = multipliers[m];
            SimConfig cfg = base_cfg;
            cfg.tof_noise_std_m *= M;
            cfg.tof_noise_pct_of_range *= M;
            cfg.tof_dropout_rate *= M;
            cfg.wheel_slip_coefficient *= M;
            cfg.encoder_noise_base *= M;
            cfg.encoder_noise_speed_coeff *= M;
            cfg.wall_noise_std_m *= M;
            cfg.sensor_angle_bias_std_deg *= M;
            cfg.wheel_scale_bias_std *= M;
            cfg.max_speed_bias_std *= M;

            TierCounts c;
            for (int trial = 0; trial < NUM_TRIALS; ++trial) {
                std::mt19937 trial_rng(2000 + trial);
                auto active_track = PerturbTrack(t.walls, cfg.wall_noise_std_m, trial_rng);

                Robot bot;
                bot.brain = brain;
                bot.Reset(Robot::SpawnAxle(cfg, t.spawn, t.spawn_heading), t.spawn_heading, cfg.tof_max_range_m);
                bot.SampleHardwareBias(cfg, trial_rng);

                int steps_run = 0;
                for (int step = 0; step < MAX_STEPS; ++step) {
                    bot.Step(active_track, cfg, trial_rng, finish_seg);
                    steps_run++;
                    if (!bot.alive || bot.finished) break;
                }

                // Three outcomes: crashed, finished, or ran out of steps (counted in neither).
                c.trials++;
                if (!bot.alive) {
                    c.crashes++;
                } else if (bot.finished) {
                    c.finishes++;
                    c.total_finish_time += steps_run * SIMULATION_DT;
                }
            }

            BenchmarkRow row = MakeRow((double)M, c);
            PrintRow(std::cout, t.name, row);
            csv_row(t.name, row);
            rows.push_back(row);

            overall[m].crashes += c.crashes;
            overall[m].finishes += c.finishes;
            overall[m].trials += c.trials;
            overall[m].total_finish_time += c.total_finish_time;
        }
        std::cout << "---------------------------------------------------------------------------------------\n";
        ExportBenchmarkXlsx(t.name, rows);
    }

    std::vector<BenchmarkRow> overall_rows;
    for (int m = 0; m < NUM_MULT; ++m) {
        BenchmarkRow row = MakeRow((double)multipliers[m], overall[m]);
        PrintRow(std::cout, "OVERALL", row);
        csv_row("OVERALL", row);
        overall_rows.push_back(row);
    }
    std::cout << "=======================================================================================\n";
    std::cout << "Benchmark results exported to benchmark_results.csv\n";
    if (tracks.size() > 1) ExportBenchmarkXlsx("overall_" + TrackSetLabel(track_names), overall_rows);

    return 0;
}
