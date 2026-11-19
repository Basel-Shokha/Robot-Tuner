#include "raylib.h"
#include "Types.hpp"
#include "Raycast.hpp"
#include "PidController.hpp"
#include "Track.hpp"
#include "Robot.hpp"
#include "SimConfig.hpp"
#include "GraphExportHelper.hpp"
#include <iostream>
#include <string>
#include <vector>
#include <deque>
#include <algorithm>
#include <cmath>
#include <cctype>
#include <filesystem>
#include "xlsxwriter.h"

struct TelemetryRecord {
    float time;
    float v_l;
    float eff_v_l;
    float v_r;
    float eff_v_r;
    float dists[3];
    float raw_center_dist;
};

void ExportRunMotorVelocity(const std::string& track_name, const std::vector<TelemetryRecord>& records) {
    if (records.empty()) return;

    std::string folder = "Graphs/visual_graphs/time_to_motor_velocity";
    std::string filepath = GraphExport::FormTrackXlsxPath(folder, track_name);

    lxw_workbook* wb = workbook_new(filepath.c_str());
    if (!wb) return;
    lxw_worksheet* ws = workbook_add_worksheet(wb, "MotorVelocity");

    lxw_format* header_fmt = workbook_add_format(wb);
    format_set_bold(header_fmt);

    worksheet_write_string(ws, 0, 0, "Time", header_fmt);
    worksheet_write_string(ws, 0, 1, "v_l_cmd", header_fmt);
    worksheet_write_string(ws, 0, 2, "v_l_eff", header_fmt);
    worksheet_write_string(ws, 0, 3, "v_r_cmd", header_fmt);
    worksheet_write_string(ws, 0, 4, "v_r_eff", header_fmt);

    for (size_t i = 0; i < records.size(); ++i) {
        lxw_row_t row = (lxw_row_t)(i + 1);
        worksheet_write_number(ws, row, 0, (double)records[i].time, NULL);
        worksheet_write_number(ws, row, 1, (double)records[i].v_l, NULL);
        worksheet_write_number(ws, row, 2, (double)records[i].eff_v_l, NULL);
        worksheet_write_number(ws, row, 3, (double)records[i].v_r, NULL);
        worksheet_write_number(ws, row, 4, (double)records[i].eff_v_r, NULL);
    }

    lxw_chart* chart = workbook_add_chart(wb, LXW_CHART_LINE);
    std::string num_rows_str = std::to_string(records.size() + 1);

    std::string cat_range = "=MotorVelocity!$A$2:$A$" + num_rows_str;
    std::string val_range_vl_cmd = "=MotorVelocity!$B$2:$B$" + num_rows_str;
    std::string val_range_vl_eff = "=MotorVelocity!$C$2:$C$" + num_rows_str;
    std::string val_range_vr_cmd = "=MotorVelocity!$D$2:$D$" + num_rows_str;
    std::string val_range_vr_eff = "=MotorVelocity!$E$2:$E$" + num_rows_str;

    lxw_chart_series* s1 = chart_add_series(chart, cat_range.c_str(), val_range_vl_cmd.c_str());
    chart_series_set_name(s1, "=MotorVelocity!$B$1");

    lxw_chart_series* s2 = chart_add_series(chart, cat_range.c_str(), val_range_vl_eff.c_str());
    chart_series_set_name(s2, "=MotorVelocity!$C$1");

    lxw_chart_series* s3 = chart_add_series(chart, cat_range.c_str(), val_range_vr_cmd.c_str());
    chart_series_set_name(s3, "=MotorVelocity!$D$1");

    lxw_chart_series* s4 = chart_add_series(chart, cat_range.c_str(), val_range_vr_eff.c_str());
    chart_series_set_name(s4, "=MotorVelocity!$E$1");

    chart_title_set_name(chart, "Commanded vs Effective Wheel Velocity");
    chart_axis_set_name(chart->x_axis, "Time (s)");
    chart_axis_set_name(chart->y_axis, "Velocity (m/s)");

    worksheet_insert_chart(ws, 1, 6, chart);
    workbook_close(wb);
    std::cout << "[GraphExport] Saved motor velocity telemetry: " << filepath << "\n";
}

void ExportRunSensorReadings(const std::string& track_name, const std::vector<TelemetryRecord>& records) {
    if (records.empty()) return;

    std::string folder = "Graphs/visual_graphs/time_to_sensor_readings";
    std::string filepath = GraphExport::FormTrackXlsxPath(folder, track_name);

    lxw_workbook* wb = workbook_new(filepath.c_str());
    if (!wb) return;
    lxw_worksheet* ws = workbook_add_worksheet(wb, "SensorReadings");

    lxw_format* header_fmt = workbook_add_format(wb);
    format_set_bold(header_fmt);

    worksheet_write_string(ws, 0, 0, "Time", header_fmt);
    worksheet_write_string(ws, 0, 1, "Left", header_fmt);
    worksheet_write_string(ws, 0, 2, "Right", header_fmt);
    worksheet_write_string(ws, 0, 3, "Center_Filtered", header_fmt);
    worksheet_write_string(ws, 0, 4, "Center_Raw", header_fmt);

    for (size_t i = 0; i < records.size(); ++i) {
        lxw_row_t row = (lxw_row_t)(i + 1);
        worksheet_write_number(ws, row, 0, (double)records[i].time, NULL);
        worksheet_write_number(ws, row, 1, (double)records[i].dists[0], NULL);
        worksheet_write_number(ws, row, 2, (double)records[i].dists[2], NULL);
        worksheet_write_number(ws, row, 3, (double)records[i].dists[1], NULL);
        worksheet_write_number(ws, row, 4, (double)records[i].raw_center_dist, NULL);
    }

    lxw_chart* chart = workbook_add_chart(wb, LXW_CHART_LINE);
    std::string num_rows_str = std::to_string(records.size() + 1);

    std::string cat_range = "=SensorReadings!$A$2:$A$" + num_rows_str;
    std::string val_range_left = "=SensorReadings!$B$2:$B$" + num_rows_str;
    std::string val_range_right = "=SensorReadings!$C$2:$C$" + num_rows_str;
    std::string val_range_center_filt = "=SensorReadings!$D$2:$D$" + num_rows_str;
    std::string val_range_center_raw = "=SensorReadings!$E$2:$E$" + num_rows_str;

    lxw_chart_series* s1 = chart_add_series(chart, cat_range.c_str(), val_range_left.c_str());
    chart_series_set_name(s1, "=SensorReadings!$B$1");

    lxw_chart_series* s2 = chart_add_series(chart, cat_range.c_str(), val_range_right.c_str());
    chart_series_set_name(s2, "=SensorReadings!$C$1");

    lxw_chart_series* s3 = chart_add_series(chart, cat_range.c_str(), val_range_center_filt.c_str());
    chart_series_set_name(s3, "=SensorReadings!$D$1");

    lxw_chart_series* s4 = chart_add_series(chart, cat_range.c_str(), val_range_center_raw.c_str());
    chart_series_set_name(s4, "=SensorReadings!$E$1");

    chart_title_set_name(chart, "ToF Sensor Readings and Glitch Filtering");
    chart_axis_set_name(chart->x_axis, "Time (s)");
    chart_axis_set_name(chart->y_axis, "Distance (m)");

    worksheet_insert_chart(ws, 1, 6, chart);
    workbook_close(wb);
    std::cout << "[GraphExport] Saved sensor telemetry: " << filepath << "\n";
}

int main(int argc, char** argv) {
    SimConfig config = LoadConfig("sim_config.txt");

    // Playlist mode: --all plays every tracks/*.track in name order, --tracks a,b,c plays those.
    // Each track runs once (finish, crash or max_episode_steps), then the next one loads; the
    // window closes after the last. With no arguments the visualizer loops view_track forever.
    std::vector<std::string> playlist;
    std::string model_path = "best_pid_model.txt";
    for (int i = 1; i < argc; ++i) {
        std::string k = argv[i];
        if (k == "--all") {
            std::vector<std::string> found;
            std::error_code ec;
            for (const auto& e : std::filesystem::directory_iterator(std::filesystem::exists("tracks") ? "tracks" : "../tracks", ec)) {
                if (e.path().extension() == ".track") found.push_back(e.path().stem().string());
            }
            std::sort(found.begin(), found.end(), [](const std::string& a, const std::string& b) {
                return std::lexicographical_compare(a.begin(), a.end(), b.begin(), b.end(),
                    [](char x, char y) { return std::tolower((unsigned char)x) < std::tolower((unsigned char)y); });
            });
            playlist.insert(playlist.end(), found.begin(), found.end());
        } else if (k == "--tracks" && i + 1 < argc) {
            std::vector<std::string> named = SplitTrackList(argv[++i]);
            playlist.insert(playlist.end(), named.begin(), named.end());
        } else if (k == "--model" && i + 1 < argc) {
            model_path = argv[++i];
        } else {
            std::cerr << "Warning: unknown argument '" << k << "' ignored.\n";
        }
    }
    const bool playlist_mode = !playlist.empty();
    if (!playlist_mode) playlist = {config.view_track};

    const int screenWidth = 1100;
    const int screenHeight = 850;
    
    InitWindow(screenWidth, screenHeight, "Autonomous 3-ToF Robot - Live Simulation Dashboard");
    SetTargetFPS(60);

    size_t track_index = 0;
    std::string current_track;
    Vec2 spawn_pos;
    float spawn_heading = 0.0f;
    FinishLine finish;
    std::vector<Wall> track;
    const Wall* finish_seg = nullptr;

    const float render_scale = 90.0f;
    Camera2D camera = {0};
    camera.offset = {screenWidth / 2.0f, screenHeight / 2.0f};
    camera.zoom = 1.1f;
    camera.rotation = 0.0f;
    bool follow_robot = true;

    auto LoadTrackAt = [&](size_t i) {
        track_index = i;
        current_track = playlist[i];
        track = GenerateTrack(current_track, spawn_pos, spawn_heading, &finish);
        finish_seg = finish.has_finish ? &finish.segment : nullptr;
        camera.target = {spawn_pos.x * render_scale, spawn_pos.y * render_scale};
        follow_robot = true;
        SetWindowTitle(TextFormat("3-ToF Robot - %s (%d/%d)", current_track.c_str(), (int)i + 1, (int)playlist.size()));
        if (playlist_mode) {
            std::cout << "[Playlist] Track " << (i + 1) << "/" << playlist.size() << ": " << current_track << "\n";
        }
    };
    LoadTrackAt(0);

    std::mt19937 rng(1234);

    Robot bot;
    bot.brain = PidBrain(config);
    bot.Reset(Robot::SpawnAxle(config, spawn_pos, spawn_heading), spawn_heading, config.tof_max_range_m);
    bot.SampleHardwareBias(config, rng);

    if (bot.brain.Load(model_path)) {
        std::cout << "Loaded checkpoint model from " << model_path << "\n";
    } else {
        std::cout << "No trained model found. Running default (untrained) PID gains.\n";
    }

    bool paused = false;
    bool step_once = false;
    float sim_speed = 1.0f;
    std::vector<Vec2> trail;
    float time_accum = 0.0f;
    
    std::deque<TelemetryRecord> telemetry_history;
    std::vector<TelemetryRecord> full_run_records;
    
    float elapsed_time = 0.0f;
    const float tiers[] = {10.0f, 25.0f, 60.0f, 90.0f, 120.0f};
    int current_window_tier = 0;

    auto HandleRunEndAndReset = [&](bool was_collision) {
        if (!full_run_records.empty() && !playlist_mode) {  // playlist runs are for watching only
            std::cout << (was_collision ? "[Reset] Collision triggered run termination.\n" 
                                        : "[Reset] User initiated manual run termination.\n");
            ExportRunMotorVelocity(current_track, full_run_records);
            ExportRunSensorReadings(current_track, full_run_records);
        }
        bot.Reset(Robot::SpawnAxle(config, spawn_pos, spawn_heading), spawn_heading, config.tof_max_range_m);
        bot.SampleHardwareBias(config, rng);  // each run is a freshly built robot
        trail.clear();
        elapsed_time = 0.0f;
        current_window_tier = 0;
        telemetry_history.clear();
        full_run_records.clear();
        time_accum = 0.0f;
    };

    // Playlist: record how this track's run ended, then load the next track (or finish up).
    bool playlist_done = false;
    float done_timer = 0.0f;
    std::string banner;
    float banner_timer = 0.0f;
    std::vector<std::string> results;
    auto AdvancePlaylist = [&](const char* outcome) {
        std::string line = current_track + ": " + outcome + " at t = " + TextFormat("%.1f", elapsed_time) + " s";
        std::cout << "[Playlist] " << line << "\n";
        results.push_back(line);
        banner = line;
        banner_timer = 2.5f;
        if (track_index + 1 >= playlist.size()) {
            playlist_done = true;
            return;
        }
        LoadTrackAt(track_index + 1);
        HandleRunEndAndReset(false);
    };

    while (!WindowShouldClose() && !(playlist_done && done_timer > 3.0f)) {
        if (playlist_mode && !playlist_done && IsKeyPressed(KEY_N)) {
            AdvancePlaylist("SKIPPED");
        }
        if (IsKeyDown(KEY_RIGHT_BRACKET)) {
            bot.brain.hw.right_angle_deg = std::min(90.0f, bot.brain.hw.right_angle_deg + 0.5f);
        }
        if (IsKeyDown(KEY_LEFT_BRACKET)) {
            bot.brain.hw.right_angle_deg = std::max(20.0f, bot.brain.hw.right_angle_deg - 0.5f);
        }
        if (IsKeyDown(KEY_EQUAL)) {
            bot.brain.hw.track_width = std::min(0.180f, bot.brain.hw.track_width + 0.0005f);
        }
        if (IsKeyDown(KEY_MINUS)) {
            bot.brain.hw.track_width = std::max(0.080f, bot.brain.hw.track_width - 0.0005f);
        }
        if (IsKeyPressed(KEY_SPACE)) {
            paused = !paused;
        }
        if (paused && IsKeyPressed(KEY_PERIOD)) {
            step_once = true;
        }
        if (IsKeyPressed(KEY_ONE)) { sim_speed = 0.25f; }
        if (IsKeyPressed(KEY_TWO)) { sim_speed = 1.0f; }
        if (IsKeyPressed(KEY_THREE)) { sim_speed = 3.0f; }

        if (IsKeyPressed(KEY_R)) {
            HandleRunEndAndReset(false);
        }

        if ((!paused || step_once) && !playlist_done) {
            if (step_once) {
                time_accum = SIMULATION_DT;
                step_once = false;
            } else {
                time_accum += GetFrameTime() * sim_speed;
            }
            
            while (time_accum >= SIMULATION_DT) {
                bot.Step(track, config, rng, finish_seg);
                elapsed_time += SIMULATION_DT;
                time_accum -= SIMULATION_DT;

                if (current_window_tier < 4 && elapsed_time > tiers[current_window_tier]) {
                    current_window_tier++;
                }

                float window_size = tiers[current_window_tier];
                while (!telemetry_history.empty() && telemetry_history.front().time < elapsed_time - window_size) {
                    telemetry_history.pop_front();
                }

                TelemetryRecord pt;
                pt.time = elapsed_time;
                pt.v_l = bot.v_l;
                pt.eff_v_l = bot.last_eff_v_l;
                pt.v_r = bot.v_r;
                pt.eff_v_r = bot.last_eff_v_r;
                pt.dists[0] = bot.dists[0];
                pt.dists[1] = bot.dists[1];
                pt.dists[2] = bot.dists[2];
                pt.raw_center_dist = bot.last_raw_center_dist;

                telemetry_history.push_back(pt);
                full_run_records.push_back(pt);
                trail.push_back(bot.pos);

                if (!bot.alive) {
                    if (playlist_mode) AdvancePlaylist("CRASHED");
                    else HandleRunEndAndReset(true);
                    break;
                }
                if (bot.finished) {
                    std::cout << "[Finish] Reached finish line at t = " << elapsed_time << " s\n";
                    if (playlist_mode) AdvancePlaylist("FINISHED");
                    else HandleRunEndAndReset(false);
                    break;
                }
                if (playlist_mode && elapsed_time >= config.MaxSteps() * SIMULATION_DT) {
                    AdvancePlaylist("TIMED OUT");
                    break;
                }
            }
        }

        Vector2 botScreenPos = {bot.pos.x * render_scale, bot.pos.y * render_scale};

        // Free camera: wheel zooms around the cursor, left-drag pans; right click re-follows the robot.
        float wheel = GetMouseWheelMove();
        if (wheel != 0.0f) {
            Vector2 mouse = GetMousePosition();
            Vector2 before = GetScreenToWorld2D(mouse, camera);
            camera.zoom = std::clamp(camera.zoom * std::pow(1.15f, wheel), 0.02f, 20.0f);
            Vector2 after = GetScreenToWorld2D(mouse, camera);
            if (!follow_robot) {  // while following, zoom stays centred on the robot
                camera.target.x += before.x - after.x;
                camera.target.y += before.y - after.y;
            }
        }
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            Vector2 d = GetMouseDelta();
            if (d.x != 0.0f || d.y != 0.0f) {
                follow_robot = false;
                camera.target.x -= d.x / camera.zoom;
                camera.target.y -= d.y / camera.zoom;
            }
        }
        if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) follow_robot = true;

        if (follow_robot) {
            camera.target.x += (botScreenPos.x - camera.target.x) * 0.1f;
            camera.target.y += (botScreenPos.y - camera.target.y) * 0.1f;
        }

        BeginDrawing();
        ClearBackground(GetColor(0x181a20ff));

        BeginMode2D(camera);

        for (size_t i = 1; i < trail.size(); ++i) {
            DrawLineEx(
                {trail[i-1].x * render_scale, trail[i-1].y * render_scale},
                {trail[i].x * render_scale, trail[i].y * render_scale},
                1.5f,
                Fade(GREEN, 0.4f)
            );
        }

        if (finish.has_finish) {
            DrawLineEx({finish.segment.p1.x * render_scale, finish.segment.p1.y * render_scale},
                       {finish.segment.p2.x * render_scale, finish.segment.p2.y * render_scale},
                       std::max(5.0f, 2.5f / camera.zoom), GetColor(0x00e676ff));
        }

        for (const auto& w : track) {
            DrawLineEx(
                {w.p1.x * render_scale, w.p1.y * render_scale},
                {w.p2.x * render_scale, w.p2.y * render_scale},
                std::max(3.5f, 1.5f / camera.zoom),  // stay visible when zoomed far out
                GetColor(0xdce0e8ff)
            );
        }

        for (int i = 0; i < 3; ++i) {
            // Draw what the sensor actually measures: its whole cone (not one centre line), at the
            // angle it really points this run (mount angle + this build's mounting error), out to
            // the reported distance. The reading is the nearest wall ANYWHERE in the cone, so the
            // arc touches whatever produced it -- often a side wall at the cone's edge while the
            // centre line still has open space ahead. Yellow = glitch filter holding a reading.
            float mount_angle_rad = (config.sensors[i].angle_deg + bot.sensor_angle_bias_deg[i]) * (PI_F / 180.0f);
            float a = bot.theta + mount_angle_rad;
            
            float world_x = bot.pos.x + std::cos(bot.theta)*config.sensors[i].x_m - std::sin(bot.theta)*config.sensors[i].y_m;
            float world_y = bot.pos.y + std::sin(bot.theta)*config.sensors[i].x_m + std::cos(bot.theta)*config.sensors[i].y_m;

            Vector2 sensorPos = { world_x * render_scale, world_y * render_scale };
            Vector2 hitPoint = {
                (world_x + std::cos(a) * bot.dists[i]) * render_scale,
                (world_y + std::sin(a) * bot.dists[i]) * render_scale
            };
            bool held = bot.glitch_hold_count[i] > 0;
            Color beam = held ? YELLOW : RED;
            float a_deg = a * 180.0f / PI_F;
            float half_deg = config.tof_cone_half_angle_rad * 180.0f / PI_F;
            float r = bot.dists[i] * render_scale;
            DrawCircleSector(sensorPos, r, a_deg - half_deg, a_deg + half_deg, 16, Fade(beam, 0.12f));
            DrawCircleSectorLines(sensorPos, r, a_deg - half_deg, a_deg + half_deg, 16, Fade(beam, 0.45f));
            DrawLineV(sensorPos, hitPoint, Fade(beam, 0.8f));
            DrawCircleV(hitPoint, 4.0f, held ? ORANGE : MAROON);
        }

        float chassis_width = config.chassis_width_m * render_scale;
        // Outline from the axle (robot origin): chassis_rear_m behind, chassis_front_m ahead.
        float chassis_length = (Robot::ChassisFront(config) + Robot::ChassisRear(config)) * render_scale;
        Rectangle chassis = {
            botScreenPos.x, botScreenPos.y,
            chassis_length, chassis_width
        };
        Vector2 origin = {Robot::ChassisRear(config) * render_scale, chassis_width * 0.5f};
        DrawRectanglePro(chassis, origin, bot.theta * 180.0f / PI_F, GetColor(0x32a852ff));

        float wheel_len = config.wheel_length_m * render_scale;
        float wheel_w = config.wheel_width_m * render_scale;
        float wheel_offset = config.wheel_offset_from_center_m * render_scale;
        
        Vector2 wheelPos1 = {
            botScreenPos.x - std::cos(bot.theta) * wheel_offset - std::sin(bot.theta) * (config.track_width_m * 0.5f * render_scale),
            botScreenPos.y - std::sin(bot.theta) * wheel_offset + std::cos(bot.theta) * (config.track_width_m * 0.5f * render_scale)
        };
        Vector2 wheelPos2 = {
            botScreenPos.x - std::cos(bot.theta) * wheel_offset + std::sin(bot.theta) * (config.track_width_m * 0.5f * render_scale),
            botScreenPos.y - std::sin(bot.theta) * wheel_offset - std::cos(bot.theta) * (config.track_width_m * 0.5f * render_scale)
        };
        
        DrawRectanglePro({wheelPos1.x, wheelPos1.y, wheel_len, wheel_w}, {wheel_len / 2.0f, wheel_w / 2.0f}, bot.theta * 180.0f / PI_F, DARKGRAY);
        DrawRectanglePro({wheelPos2.x, wheelPos2.y, wheel_len, wheel_w}, {wheel_len / 2.0f, wheel_w / 2.0f}, bot.theta * 180.0f / PI_F, DARKGRAY);

        DrawLine(
            botScreenPos.x, botScreenPos.y,
            botScreenPos.x + std::cos(bot.theta) * 20.0f,
            botScreenPos.y + std::sin(bot.theta) * 20.0f,
            WHITE
        );

        EndMode2D();

        DrawRectangle(20, 20, 340, 268, Fade(BLACK, 0.82f));
        DrawRectangleLines(20, 20, 340, 268, DARKGRAY);

        DrawText("HARDWARE & SENSOR TELEMETRY", 35, 30, 14, YELLOW);
        DrawText(TextFormat("Side ToF Angles: L:%.1f R:%.1f deg", config.sensors[0].angle_deg, config.sensors[2].angle_deg), 35, 50, 14, RAYWHITE);
        DrawText(TextFormat("Axle Width     : %.1f mm (- / =)", bot.brain.hw.track_width * 1000.0f), 35, 70, 14, RAYWHITE);
        DrawText(TextFormat("Left ToF (m)   : %.2f", bot.dists[0]), 35, 95, 14, RED);
        DrawText(TextFormat("Center ToF (m) : %.2f (Raw: %.2f)", bot.dists[1], bot.last_raw_center_dist), 35, 115, 14, ORANGE);
        DrawText(TextFormat("Right ToF (m)  : %.2f", bot.dists[2]), 35, 135, 14, PINK);
        DrawText(TextFormat("Motors (m/s)   : L:%.2f  R:%.2f", bot.v_l, bot.v_r), 35, 160, 14, SKYBLUE);
        DrawText(TextFormat("Sim Speed      : %.2f x (1:0.25, 2:1.0, 3:3.0)", sim_speed), 35, 185, 12, GREEN);
        DrawText("[SPACE] Pause  [.] Step  [R] Reset  (yellow beam = held)", 35, 208, 12, GRAY);
        DrawText(TextFormat("Wheel: zoom (%.2fx)  L-drag: pan  R-click: %s", camera.zoom,
                            follow_robot ? "following" : "follow robot"), 35, 226, 12, GRAY);
        DrawText(TextFormat("PID  Kp:%.3f  Ki:%.4f  Kd:%.4f  cruise:%.0f%%", bot.brain.kp, bot.brain.ki,
                            bot.brain.kd, bot.brain.base_speed_frac * 100.0f), 35, 246, 12, YELLOW);

        DrawRectangle(730, 530, 340, 290, Fade(BLACK, 0.82f));
        DrawRectangleLines(730, 530, 340, 290, DARKGRAY);
        DrawText("GRAPHICAL TELEMETRY HISTORY", 745, 540, 14, YELLOW);

        DrawRectangle(740, 560, 320, 100, GetColor(0x0e1012ff));
        DrawRectangleLines(740, 560, 320, 100, DARKGRAY);
        DrawText("Velocity (Solid: Command, Dim: Effective)", 745, 665, 11, GRAY);

        DrawRectangle(740, 700, 320, 100, GetColor(0x0e1012ff));
        DrawRectangleLines(740, 700, 320, 100, DARKGRAY);
        DrawText("ToF: L(Red) R(Pink) C-Filt(Orange) C-Raw(White)", 745, 805, 11, GRAY);

        float window_size = tiers[current_window_tier];
        float min_t = (elapsed_time < window_size) ? 0.0f : (elapsed_time - window_size);

        if (telemetry_history.size() > 1) {
            for (size_t i = 1; i < telemetry_history.size(); ++i) {
                const auto& p1 = telemetry_history[i - 1];
                const auto& p2 = telemetry_history[i];
                
                float x1 = 740 + ((p1.time - min_t) / window_size) * 320.0f;
                float x2 = 740 + ((p2.time - min_t) / window_size) * 320.0f;
                
                auto vel_y = [](float v) -> float {
                    float norm = (v + 0.5f) / 1.0f;
                    return 660.0f - norm * 100.0f;
                };
                DrawLineV({x1, vel_y(p1.v_l)}, {x2, vel_y(p2.v_l)}, GetColor(0x32a852ff));
                DrawLineV({x1, vel_y(p1.eff_v_l)}, {x2, vel_y(p2.eff_v_l)}, GetColor(0x8ae6a0ff));
                
                DrawLineV({x1, vel_y(p1.v_r)}, {x2, vel_y(p2.v_r)}, GetColor(0x3280a8ff));
                DrawLineV({x1, vel_y(p1.eff_v_r)}, {x2, vel_y(p2.eff_v_r)}, GetColor(0x8ac7e6ff));
                
                auto dist_y = [&](float d) -> float {
                    float norm = d / config.tof_max_range_m;
                    return 800.0f - norm * 100.0f;
                };
                DrawLineV({x1, dist_y(p1.dists[0])}, {x2, dist_y(p2.dists[0])}, RED);
                DrawLineV({x1, dist_y(p1.dists[2])}, {x2, dist_y(p2.dists[2])}, PINK);
                DrawLineV({x1, dist_y(p1.dists[1])}, {x2, dist_y(p2.dists[1])}, ORANGE);
                DrawLineV({x1, dist_y(p1.raw_center_dist)}, {x2, dist_y(p2.raw_center_dist)}, Fade(WHITE, 0.6f));
            }
        }

        if (playlist_mode) {
            DrawText(TextFormat("Track %d/%d: %s", (int)track_index + 1, (int)playlist.size(), current_track.c_str()),
                     380, 22, 20, YELLOW);
            DrawText("[N] next track", 380, 46, 14, GRAY);
        }
        if (banner_timer > 0.0f) {
            banner_timer -= GetFrameTime();
            DrawText(banner.c_str(), 380, 66, 20, banner.find("FINISHED") != std::string::npos ? GREEN : ORANGE);
        }
        if (playlist_done) {
            done_timer += GetFrameTime();
            DrawText("ALL TRACKS DONE", screenWidth / 2 - 150, screenHeight / 2 - 20, 36, GREEN);
        }

        if (paused) {
            DrawRectangle(0, 0, screenWidth, screenHeight, Fade(BLACK, 0.5f));
            DrawText("PAUSED", screenWidth / 2 - 80, screenHeight / 2 - 20, 40, RED);
            DrawText("Press SPACE to Resume or '.' to Step", screenWidth / 2 - 160, screenHeight / 2 + 30, 20, RAYWHITE);
        }

        EndDrawing();
    }

    CloseWindow();
    if (playlist_mode) {
        std::cout << "\n[Playlist] Summary (" << results.size() << "/" << playlist.size() << " tracks run):\n";
        for (const auto& r : results) std::cout << "  " << r << "\n";
    }
    return 0;
}