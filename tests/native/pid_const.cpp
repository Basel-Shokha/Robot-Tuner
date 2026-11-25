// Unit test of the constant-cruise-speed PID (sim/rt_engine.hpp Pid::Control, pid_speed_mode 1):
//   e = clamp(d_left - d_right, +-error_clip); integral = clamp(integral + e dt, +-integral_limit);
//   d_filt = alpha d_raw + (1 - alpha) d_filt; turn = Kp e + Ki integral + Kd d_filt;
//   base = v_min + speed_frac (v_max - v_min); v_left = base - turn, v_right = base + turn, each clamped to +-v_max.
#include "../../sim/rt_api.cpp"
#include <cstdio>

static int fails = 0;
static void check(bool ok, const char* what) { if (!ok) { std::printf("FAIL %s\n", what); ++fails; } }
static bool near(float a, float b) { return std::fabs(a - b) <= 1e-6f * std::max(1.0f, std::fabs(b)); }

static Pid make(float kp, float ki, float kd, float frac, float vmin, float vmax = 0.5f) {
    Config c;
    c.n_sensors = 3;
    c.sensors[0].angle_deg = -45.0f; c.sensors[1].angle_deg = 0.0f; c.sensors[2].angle_deg = 45.0f;
    c.kp = kp; c.ki = ki; c.kd = kd;
    c.pid_error_clip_m = 0.30f; c.pid_integral_limit = 0.50f; c.pid_d_filter_alpha = 0.30f;
    c.pid_speed_mode = 1; c.pid_speed_frac = frac; c.pid_v_min = vmin; c.max_linear_speed_mps = vmax;
    Pid p; p.Configure(c); p.Reset();
    return p;
}
// d[] is indexed like the sensors: 0 = right (-45), 1 = front, 2 = left (+45)
static void tick(Pid& p, float left, float right, float dt, float& vl, float& vr) { float d[3] = {right, 0.3f, left}; p.Control(d, dt, vl, vr); }

int main() {
    float vl, vr;
    {   // centred: no turn, both wheels at base = v_min + speed_frac (v_max - v_min)
        Pid p = make(2, 0, 0, 0.75f, 0.1f);
        tick(p, 0.2f, 0.2f, 0.025f, vl, vr);
        float base = 0.1f + 0.75f * (0.5f - 0.1f);
        check(near(p.base, base) && near(vl, base) && near(vr, base), "centred: v_l = v_r = base");
        check(near(p.base, 0.4f), "base = 0.1 + 0.75 x 0.4 = 0.4");
    }
    {   // a turn does not slow the robot down (no slowdown rule): base stays constant
        Pid p = make(2, 0, 0, 0.75f, 0.1f);
        tick(p, 0.22f, 0.20f, 0.025f, vl, vr);
        float e = 0.22f - 0.20f, turn = 2.0f * e, base = 0.1f + 0.75f * 0.4f;
        check(near(p.turn, turn), "turn = Kp e");
        check(near(p.base, base), "base unchanged while turning");
        check(near(vl, base - turn) && near(vr, base + turn), "v_l = base - turn, v_r = base + turn");
        check(!p.clip_l && !p.clip_r && !p.sat, "no clipping for a small turn");
    }
    {   // clipping: base + turn above v_max is cut, the real turn is smaller than the asked one
        Pid p = make(2, 0, 0, 0.75f, 0.1f);
        tick(p, 0.40f, 0.20f, 0.025f, vl, vr);
        float turn = 2.0f * 0.2f, base = 0.4f;
        check(near(p.turn, turn), "asked turn is not clamped");
        check(near(vr, 0.5f) && near(vl, base - turn), "right wheel clamped to v_max, left = base - turn");
        check(p.clip_r && !p.clip_l, "clip flag on the right wheel only");
        check(near(p.real_turn, 0.5f * (vr - vl)) && p.real_turn < turn, "real turn < asked turn");
        check(p.sat, "saturated");
        // the same gains at speed_frac 1.0: base = v_max, so ANY turn clips one wheel
        Pid q = make(2, 0, 0, 1.0f, 0.1f);
        tick(q, 0.21f, 0.20f, 0.025f, vl, vr);
        check(near(q.base, 0.5f) && near(vr, 0.5f) && q.clip_r, "speed_frac 1: base = v_max, the outer wheel clips");
        Pid z = make(2, 0, 0, 0.0f, 0.1f);
        tick(z, 0.2f, 0.2f, 0.025f, vl, vr);
        check(near(z.base, 0.1f), "speed_frac 0: base = v_min");
    }
    {   // a huge turn: both wheels clamp to +-v_max (base - turn below -v_max)
        Pid p = make(10, 0, 0, 0.5f, 0.0f);
        tick(p, 0.40f, 0.10f, 0.025f, vl, vr);   // e clipped to 0.3 -> turn 3
        check(near(p.err, 0.30f), "error clipped to +-error_clip");
        check(near(vl, -0.5f) && near(vr, 0.5f) && p.clip_l && p.clip_r, "both wheels clamped to +-v_max");
    }
    {   // integral with anti-windup
        Pid p = make(0, 1, 0, 0.75f, 0.0f);
        for (int k = 0; k < 400; ++k) tick(p, 0.40f, 0.10f, 0.025f, vl, vr);  // 0.3 x 0.025 x 400 = 3 > 0.5
        check(near(p.integral, 0.5f), "integral clamped to +-integral_limit");
        check(near(p.turn, 0.5f), "turn = Ki x integral");
    }
    {   // derivative, low-pass filtered: at dt = 50 ms alpha = d_filter_alpha
        Pid p = make(0, 0, 1, 0.75f, 0.0f);
        tick(p, 0.20f, 0.20f, 0.05f, vl, vr);   // e = 0, first tick: no derivative
        check(near(p.d_filtered, 0.0f), "first tick: d = 0");
        tick(p, 0.21f, 0.20f, 0.05f, vl, vr);   // e = 0.01, raw d = 0.2
        float raw = (0.21f - 0.20f) / 0.05f;
        check(near(p.d_filtered, 0.3f * raw), "d_filt = alpha d_raw + (1 - alpha) d_filt");
        tick(p, 0.21f, 0.20f, 0.05f, vl, vr);   // raw d = 0
        check(near(p.d_filtered, 0.7f * 0.3f * raw), "filter decays");
        check(near(p.turn, p.d_filtered), "turn = Kd d_filt");
    }
    {   // mode 0 (upstream PidBrain) is unchanged: base slows down while turning
        Config c; c.n_sensors = 3; c.sensors[0].angle_deg = -45; c.sensors[1].angle_deg = 0; c.sensors[2].angle_deg = 45;
        c.kp = 2; c.ki = 0; c.kd = 0; c.base_speed_frac = 1.0f; c.pid_slowdown_frac = 0.5f; c.max_linear_speed_mps = 0.5f;
        Pid p; p.Configure(c); p.Reset();
        tick(p, 0.25f, 0.20f, 0.025f, vl, vr);
        check(near(p.base, 0.5f * (1.0f - 0.5f * 0.1f / 0.5f)), "mode 0: upstream slowdown rule kept");
    }
    std::printf(fails ? "pid_const: %d checks failed\n" : "pid_const: all checks passed\n", fails);
    return fails ? 1 : 0;
}
