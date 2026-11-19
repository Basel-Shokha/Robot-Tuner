// wheel_pid.h - the per-wheel speed controller and its step-response objective.
//
// ONE implementation, included unchanged by
//   - the firmware (robot_fw.ino wheelTask and CAL STEP / CAL AUTOTUNE),
//   - the simulator (*/Robot.hpp, wheel_model pid) via #include "../firmware/robot_fw/wheel_pid.h",
//   - tools/wheelsim/tune_wheel_pid.cpp and tools/equiv/equiv_test.cpp.
// Portable C++: no Arduino headers, no heap.
//
// Units: wheel speed in encoder ticks PER PID PERIOD ("tp", 10 ms by default), output in PWM counts
// (forward positive). A plain discrete PID with feedforward:
//   pwm = ff_static*sign(target) + ff_gain*target + Kp*e + Ki*sum(e) + Kd*(e - e_prev)
// A |target| below 0.01 tp gives zero PWM and clears the state (no chatter around a deadband).
// Anti-windup: the integral only accumulates while the output is not clamped.
//
// Gains depend on the period. To move gains from period A to period B with the same continuous-time
// behaviour (r = B / A; speeds in ticks/B are r times speeds in ticks/A):
//   Kp_B = Kp_A / r      ff_gain_B = ff_gain_A / r      ff_static_B = ff_static_A
//   Ki_B = Ki_A          (r times more samples, each error r times smaller)
//   Kd_B = Kd_A / r^2    (each sample's error change is r^2 times smaller)
// ConvertGains() does exactly this. Tom's 50 ms gains (0.4 / 0.001 / 0.05, ff 35 / 2.0) become, at 10 ms,
// 2.0 / 0.001 / 1.25, ff 35 / 10.0.
#pragma once
#include <math.h>

namespace fw {

struct WheelPidGains {
    float kp = 0.4f, ki = 0.001f, kd = 0.05f;
    float ff_static = 35.0f, ff_gain = 2.0f;
    // Reverse feedforward (< 0 = same as forward). Tom's drive scheme brakes the motor during the PWM off-time
    // in one direction and lets it coast in the other, so the speed per PWM can differ by direction (CAL FF REV).
    float ff_static_rev = -1.0f, ff_gain_rev = -1.0f;
    float pwm_limit = 120.0f;   // clamp, PWM counts
    float pwm_max = 255.0f;     // hardware maximum
};

struct WheelPid {
    float integ = 0.0f, prev_e = 0.0f;

    void Reset() { integ = 0.0f; prev_e = 0.0f; }

    // One PID period. target / meas in ticks per PID period. Returns the PWM (rounded).
    int Step(const WheelPidGains& g, bool enabled, float target, float meas) {
        if (!enabled || fabsf(target) < 0.01f) { Reset(); return 0; }
        float e = target - meas;
        float fs = target > 0 || g.ff_static_rev < 0 ? g.ff_static : g.ff_static_rev;
        float fg = target > 0 || g.ff_gain_rev < 0 ? g.ff_gain : g.ff_gain_rev;
        float ff = (target > 0 ? fs : -fs) + fg * target;
        float trial_i = integ + e;
        float u = ff + g.kp * e + g.ki * trial_i + g.kd * (e - prev_e);
        float lim = g.pwm_limit < 0.0f ? 0.0f : (g.pwm_limit > g.pwm_max ? g.pwm_max : g.pwm_limit);
        if (u > lim) u = lim;
        else if (u < -lim) u = -lim;
        else integ = trial_i;  // anti-windup
        prev_e = e;
        return (int)lroundf(u);
    }
};

// Gains for period_b from gains for period_a with the same continuous-time behaviour (see the top).
static inline WheelPidGains ConvertGains(WheelPidGains g, float period_a_ms, float period_b_ms) {
    float r = period_b_ms / period_a_ms;
    g.kp /= r; g.ff_gain /= r; g.kd /= (r * r);
    if (g.ff_gain_rev >= 0) g.ff_gain_rev /= r;
    return g;
}

// Unit conversions shared by firmware and sim. m_per_tick = pi * diameter / ticks_per_rev * wheel_scale.
static inline float MetersPerTick(float diameter_m, float ticks_per_rev, float wheel_scale) {
    return 3.14159265359f * diameter_m / ticks_per_rev * wheel_scale;
}
// Speed in ticks per period (tp) <-> m/s. period_s = the PID period (or 0.05 for Tom's ticks/50 ms).
static inline float MpsToTp(float v_mps, float m_per_tick, float period_s) { return v_mps * period_s / m_per_tick; }
static inline float TpToMps(float tp, float m_per_tick, float period_s) { return tp * m_per_tick / period_s; }

// Step-response metrics of one wheel: samples y[i] (any speed unit, same as tgt) at times t[i] (s, t[0] = step instant), target tgt.
struct StepMetrics { float rise = -1, over = -1, tau = -1, ss = 0; };
static inline StepMetrics ComputeStepMetrics(const float* t, const float* y, int n, float tgt) {
    StepMetrics m;
    if (n < 6) return m;
    int third = n / 3;
    for (int i = n - third; i < n; ++i) m.ss += y[i];
    m.ss /= third;
    float peak = 0, t10 = -1, t90 = -1, t63 = -1;
    for (int i = 0; i < n; ++i) {
        float v = y[i] / tgt;
        if (v > peak) peak = v;
        if (t10 < 0 && v >= 0.1f) t10 = t[i];
        if (t63 < 0 && v >= 0.632f) t63 = t[i];
        if (t90 < 0 && v >= 0.9f) t90 = t[i];
    }
    m.rise = (t10 >= 0 && t90 >= 0) ? t90 - t10 : -1;
    m.tau = t63 >= 0 ? t63 - t[0] : -1;
    m.over = peak > 1.0f ? (peak - 1.0f) * 100.0f : 0.0f;
    return m;
}
// The auto-tune objective (firmware CAL AUTOTUNE and tools/wheelsim): rise time, plus a heavy penalty for
// overshoot above 2 % and for steady-state error. Lower is better; a step that never reaches 90 % costs 2 s.
static inline float StepCost(const StepMetrics& m, float tgt) {
    return (m.rise < 0 ? 2.0f : m.rise) + 0.05f * (m.over > 2.0f ? m.over - 2.0f : 0.0f) + 0.5f * fabsf(m.ss - tgt) / tgt;
}

}  // namespace fw
