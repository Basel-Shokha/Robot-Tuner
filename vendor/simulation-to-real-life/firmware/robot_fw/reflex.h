// reflex.h - "side wall too close" reflex (Nicole, 6 Nov 2026), shared by the simulator and (later) the firmware.
// BENCHMARK ONLY for now: off by default (reflex_on 0), not used in training, not in any firmware build yet.
//
// When the left or right ToF (after the glitch filter) reads <= trigger_m: HALT (both wheels 0 for halt_s), then
// ROTATE IN PLACE by turn_rad AWAY from that wall (right sensor close -> turn left, left close -> turn right), the
// angle measured from the encoders, then hand back to the controller (the caller resets the steering smoothing).
// Forward speed stays >= 0 throughout (turning in place: one wheel forward, one back, forward speed 0).
#pragma once
#include <math.h>

namespace reflex {

struct Params {
    int enabled = 0;
    float trigger_m = 0.09f;     // side reading at or below this starts the reflex
    float halt_s = 0.10f;        // standing still before the turn
    float turn_rad = 0.785398f;  // 45 degrees
    float turn_speed = 0.08f;    // wheel speed while turning in place, m/s
    float max_s = 2.0f;          // safety: give up the turn after this long
    float track_w = 0.1242f;     // for the encoder angle
};

enum State { IDLE = 0, HALT = 1, TURN = 2 };

struct Logic {
    Params p;
    State state = IDLE;
    int dir = 0;          // +1 turn left (CCW), -1 turn right
    float t = 0.0f, rot = 0.0f;
    int triggers = 0;     // how often it fired in this run (benchmark statistics)
    void Reset() { state = IDLE; dir = 0; t = 0; rot = 0; triggers = 0; }

    // dists[0] = right, [2] = left (as everywhere). enc_* = encoder speeds of the last tick (m/s).
    // Returns true while the reflex drives (v_l / v_r set); ended = true on the tick it hands back.
    bool Step(const float dists[3], float enc_l, float enc_r, float dt, float& v_l, float& v_r, bool& ended) {
        ended = false;
        if (!p.enabled) { state = IDLE; return false; }
        if (state == IDLE) {
            bool r = dists[0] <= p.trigger_m, l = dists[2] <= p.trigger_m;
            if (!r && !l) return false;
            dir = (r && (!l || dists[0] <= dists[2])) ? +1 : -1;  // the closer wall wins
            state = HALT; t = 0; rot = 0; ++triggers;
        }
        t += dt;
        if (state == TURN) rot += (enc_r - enc_l) / p.track_w * dt;  // rotation of the tick just driven
        if (state == HALT && t >= p.halt_s) { state = TURN; t = 0; rot = 0; }
        if (state == TURN && (fabsf(rot) >= p.turn_rad || t >= p.max_s)) {
            state = IDLE; ended = true; return false;
        }
        if (state == HALT) { v_l = 0; v_r = 0; }
        else { v_l = -dir * p.turn_speed; v_r = dir * p.turn_speed; }
        return true;
    }
};

// Variant 2 (benchmark only): SLOW DOWN near a side wall instead of turning. When the left or right ToF reads
// <= slow_m, both wheel commands of the controller are scaled by slow_frac (same curve, lower speed); full speed again
// as soon as both sides read more than slow_m. Applied to the controller output, before the steering smoothing.
struct Slow {
    int enabled = 0;
    float slow_m = 0.10f;
    float slow_frac = 0.5f;
    int keep_turn = 0;      // 1: scale only the forward speed, keep the turn -> a SHARPER curve near the wall
    long slowed_ticks = 0;  // statistics
    void Reset() { slowed_ticks = 0; }
    void Apply(const float dists[3], float& v_l, float& v_r) {
        if (!enabled || (dists[0] > slow_m && dists[2] > slow_m)) return;
        if (keep_turn) {
            float turn = 0.5f * (v_r - v_l), lin = 0.5f * (v_r + v_l) * slow_frac;
            v_l = lin - turn; v_r = lin + turn;
        } else {
            v_l *= slow_frac; v_r *= slow_frac;
        }
        ++slowed_ticks;
    }
};

}  // namespace reflex
