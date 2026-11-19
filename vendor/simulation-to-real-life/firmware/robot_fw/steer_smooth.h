// steer_smooth.h - smooth wheel commands for NN, PID and LineFit, shared by the simulator and the firmware.
//
// On the real robot the controllers' steering jumped from tick to tick (violent left/right twitches). Every
// controller output now goes through this filter before it reaches the wheel PID, in the sim (*/Robot.hpp) and on
// the robot (controllers.h RobotLogic), so training sees exactly the smoothed robot it will drive:
//   turn = (v_r - v_l)/2 and speed = (v_l + v_r)/2 are separated;
//   turn  : first-order low-pass (time constant turn_tau_s), then rate-limited to turn_accel (m/s per second);
//   speed : rate-limited to lin_accel (m/s per second);
//   then each WHEEL may change by at most max_step m/s per control tick (default 0.01 m/s: gentle, even steps).
// The emergency manoeuvre bypasses it (it must act at once); the filter restarts from standstill afterwards.
#pragma once
#include <math.h>

namespace smooth {

struct Params {
    int enabled = 1;
    float turn_tau_s = 0.10f;   // low-pass on the turn command
    float turn_accel = 0.6f;    // max change of the turn component, m/s per s (0 -> full 0.156 turn in ~0.26 s)
    float lin_accel = 0.5f;     // max change of the forward speed, m/s per s
    float max_step = 0.01f;     // max change of each wheel target per control tick, m/s (0 = off)
};

struct Filter {
    Params p;
    float turn = 0.0f, lin = 0.0f, out_l = 0.0f, out_r = 0.0f;
    void Reset() { turn = 0.0f; lin = 0.0f; out_l = 0.0f; out_r = 0.0f; }
    void Apply(float& v_l, float& v_r, float dt) {
        if (!p.enabled) return;
        float want_t = 0.5f * (v_r - v_l), want_v = 0.5f * (v_r + v_l);
        float a = p.turn_tau_s > 0.0f ? 1.0f - expf(-dt / p.turn_tau_s) : 1.0f;
        float lp = turn + a * (want_t - turn);
        float dmax = p.turn_accel * dt;
        turn += fminf(fmaxf(lp - turn, -dmax), dmax);
        float vmax = p.lin_accel * dt;
        lin += fminf(fmaxf(want_v - lin, -vmax), vmax);
        v_l = lin - turn;
        v_r = lin + turn;
        if (p.max_step > 0.0f) {  // per-wheel step limit; the filter state follows what was really sent
            v_l = out_l + fminf(fmaxf(v_l - out_l, -p.max_step), p.max_step);
            v_r = out_r + fminf(fmaxf(v_r - out_r, -p.max_step), p.max_step);
            lin = 0.5f * (v_l + v_r);
            turn = 0.5f * (v_r - v_l);
        }
        out_l = v_l; out_r = v_r;
    }
};

}  // namespace smooth
