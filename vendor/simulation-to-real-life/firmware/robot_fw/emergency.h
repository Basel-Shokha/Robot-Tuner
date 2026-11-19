// emergency.h - emergency manoeuvre for 20-25 cm corridors, shared by the simulator and the firmware.
//
// The old emergency behaviour (scan: spin +90, -90 deg in place, then face the more open side) needs a
// 313 mm wide corridor (turn-in-place diameter of the 180 x 140 mm body about its rear axle). The real
// corridors are 200-250 mm wide, so it hits a wall every time. This replaces it with something that fits:
//
//   NAVIGATING --(front < front_m  OR  a side < side_m)--> STOP --(stop_s)--> REVERSE --(back_m or max_s)--> NAVIGATING
//
//   STOP     : both wheel targets 0 for stop_s (lets the wheel PID brake from cruise speed).
//   REVERSE  : drive backwards at back_speed while turning the nose AWAY from the closer wall
//              (closer = the shorter of the two side readings at the trigger), at a turn rate of
//              turn_frac * back_speed difference between the wheels. Ends after back_m of travel
//              (encoders) or turn_max_rad of rotation (encoders) or max_s, whichever comes first.
//   then the controller takes over again (its state is reset by the caller, like after the old scan).
//
// Reversing keeps the rear (only 40 mm behind the axle) inside the corridor: the rotation about the axle
// is limited to turn_max_rad, so the body's sweep stays well under the corridor width.
// Header-only, no Arduino headers, no heap: compiled into the firmware (controllers.h), every simulator
// folder (*/Robot.hpp) and tools/equiv/equiv_test.cpp. One file, so sim and robot run the same code.
#pragma once
#include <math.h>

namespace emerg {

// NEVER REVERSE (Nov 2026): the robot must never drive backwards in an autonomous mode. If a controller asks for
// a negative forward speed (mean of the two wheels < 0; the NN can do that), both wheels are raised by the same
// amount so the forward speed is 0 and the steering (wheel difference) is kept. One wheel may still turn
// backwards in a sharp turn; the body does not move backwards.
inline void NoReverse(float& v_l, float& v_r) {
    float m = 0.5f * (v_l + v_r);
    if (m < 0.0f) { v_l -= m; v_r -= m; }
}

enum State { NAV = 0, STOP = 1, REVERSE = 2 };

struct Params {
    int enabled = 1;
    float front_m = 0.12f;       // trigger: front reading below this
    float side_m = 0.075f;       // trigger: either 45 deg side reading below this (0 = off). Centred and
                                 // parallel in a 20 cm corridor the side sensors read ~127 mm; touching a
                                 // wall with the body they read ~85 mm.
    float stop_s = 0.10f;        // standstill before reversing
    float back_speed = 0.08f;    // m/s, reverse speed (about half of the 0.156 m/s maximum)
    float turn_frac = 0.6f;      // wheel speed difference while reversing = turn_frac * back_speed
    float back_m = 0.06f;        // reverse at most this far (m, from the encoders)
    float turn_max_rad = 0.35f;  // rotate at most this much while reversing (~20 deg)
    float max_s = 1.5f;          // hard time limit of the reverse phase
    float track_w = 0.12f;       // track width the rotation is computed with (= the controller's belief)
};

struct Logic {
    Params p;
    int state = NAV;
    float t = 0.0f;           // time in the current state
    float dist = 0.0f;        // reversed distance (m, positive)
    float rot = 0.0f;         // rotation since the reverse started (rad, signed)
    int turn_dir = 0;         // +1: nose turns left (CCW), -1: nose turns right
    int count = 0;            // emergencies in this run
    // Free space behind the robot: metres driven FORWARD since the start or the last reverse. There is no rear
    // sensor, so the robot may only back up into space it just came from (it used to reverse into the wall behind
    // it right after a start close to the track's back wall).
    float room = 0.0f;
    float prev_l = 0.0f, prev_r = 0.0f;  // encoder speeds of the previous tick, for the forward distance
    bool motion = false;      // the previous tick's command was an emergency command

    void Reset() { state = NAV; t = dist = rot = 0.0f; turn_dir = 0; count = 0; motion = false; room = 0.0f; }

    // d: filtered readings (0 right, 1 front, 2 left), enc_*: measured wheel speeds of the last tick (m/s).
    // Returns true when the emergency owns the wheels this tick (v_l / v_r set); false = run the controller.
    // just_ended is set on the tick the manoeuvre finishes, so the caller can reset its controller state.
    bool Step(const float d[3], float enc_l, float enc_r, float dt, float& v_l, float& v_r, bool& just_ended) {
        just_ended = false;
        if (motion) {  // integrate the motion that the previous emergency command produced
            dist += -0.5f * (enc_l + enc_r) * dt;
            rot += (enc_r - enc_l) / p.track_w * dt;
        }
        if (!motion) room = fminf(room + fmaxf(0.0f, 0.5f * (enc_l + enc_r)) * dt, 1.0f);
        if (state == NAV) {
            // Not enough clear space behind: no manoeuvre (the controller keeps steering away by itself).
            if (room < 0.02f) { motion = false; return false; }
            bool trig = p.enabled && (d[1] < p.front_m || (p.side_m > 0.0f && (d[0] < p.side_m || d[2] < p.side_m)));
            if (!trig) { motion = false; return false; }
            state = STOP; t = 0.0f; dist = 0.0f; rot = 0.0f; ++count;
            // The closer wall is on the side with the shorter reading: turn the nose away from it.
            turn_dir = d[2] < d[0] ? -1 : 1;
        }
        t += dt;
        if (state == STOP) {
            v_l = v_r = 0.0f;
            if (t >= p.stop_s) { state = REVERSE; t = 0.0f; dist = 0.0f; rot = 0.0f; }
            motion = false;  // this tick commands 0: nothing to integrate next tick
            return true;
        }
        // REVERSE
        if (dist >= fminf(p.back_m, room - 0.01f) || fabsf(rot) >= p.turn_max_rad || t > p.max_s) {
            state = NAV; just_ended = true; motion = false;
            room = 0.0f;  // the reversed space is used up
            return false;  // the controller drives this tick
        }
        // Nose left (CCW, omega > 0) while backing: right wheel less negative than left.
        float h = 0.5f * p.turn_frac * p.back_speed * (float)turn_dir;
        v_l = -p.back_speed - h;
        v_r = -p.back_speed + h;
        motion = true;
        return true;
    }
};

}  // namespace emerg
