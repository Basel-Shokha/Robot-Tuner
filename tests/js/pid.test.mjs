// The constant-cruise-speed PID across the site: robot defaults, engine config, training, exported C code
// (compiled with g++ and checked against the engine's own Pid::Control), the generated sketch, and CAL VMIN.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { execFileSync } from 'node:child_process';
import { mkdtempSync, writeFileSync, rmSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { defaultRobot, migrate, robotToConfig, cruiseSpeed, robotToHardwareTxt, hardwareTxtToRobot } from '../../web/js/store.js';
import { grid, mutate, rngJS, episodeConfig } from '../../web/js/train-core.js';
import { cFunction, configH, modelTxt, simConfigLines } from '../../web/js/export.js';
import { generateSketch } from '../../web/js/firmware-gen.js';
import { mergeEnv } from '../../web/js/corridor.js';
import * as P from '../../web/js/measure-steps.js';

const root = new URL('../../', import.meta.url).pathname;

test('base = v_min + speed_frac (v_max - v_min); speed_frac defaults to 0.75', () => {
  const r = defaultRobot();
  assert.equal(r.pid.speed_frac, 0.75); assert.equal(r.motor.v_min_mps, 0.013);  // upstream cs_speed_frac, cs_v_min_mps
  assert.equal(r.pid.speed_mps, undefined); assert.equal(r.pid.slowdown_frac, undefined);
  r.motor.max_speed_mps = 0.5; r.motor.v_min_mps = 0.1;
  assert.ok(Math.abs(cruiseSpeed(r) - 0.4) < 1e-12);
  assert.ok(Math.abs(cruiseSpeed(r, 1) - 0.5) < 1e-12 && Math.abs(cruiseSpeed(r, 0) - 0.1) < 1e-12);
  // an old robot file (cruise speed in m/s, slowdown rule) gets the default speed_frac
  const old = JSON.parse(JSON.stringify(defaultRobot())); delete old.pid.speed_frac; old.pid.speed_mps = 0.12; old.pid.slowdown_frac = 0.5;
  const m = migrate(old);
  assert.equal(m.pid.speed_frac, 0.75); assert.equal(m.pid.speed_mps, undefined); assert.equal(m.pid.slowdown_frac, undefined);
  // the engine runs the constant-speed mode with the robot's speed_frac and v_min
  const c = robotToConfig(r);
  assert.equal(c.pid_speed_mode, 1); assert.equal(c.pid_speed_frac, 0.75); assert.equal(c.pid_v_min, 0.1);
  assert.equal(c.pid_slowdown_frac, undefined);
  // v_min and speed_frac survive robot_hardware.txt
  const b = hardwareTxtToRobot(robotToHardwareTxt({ ...r, pid: { ...r.pid, speed_frac: 0.6 } }));
  assert.equal(b.motor.v_min_mps, 0.1); assert.equal(b.pid.speed_frac, 0.6);
});

test('training uses the chosen speed_frac and never changes it', () => {
  const r = defaultRobot(), env = mergeEnv({ width_mm: 200 });
  const g = grid(r, env, 0.6);
  assert.ok(g.length > 10 && g.every((c) => c.speed === 0.6));
  const rng = rngJS(3);
  let c = g[7];
  for (let k = 0; k < 200; k++) { c = mutate(c, rng); assert.equal(c.speed, 0.6); }
  const cfg = episodeConfig(r, { ...env, start: { angle_deg: 20 }, noise: { global: 1 } }, { kp: 1, ki: 0, kd: 0.1, speed: 0.6 });
  assert.equal(cfg.pid_speed_frac, 0.6); assert.equal(cfg.pid_speed_mode, 1);
});

test('exported pid_gains.h computes exactly what the engine computes (compiled with g++)', (t) => {
  try { execFileSync('g++', ['--version']); } catch (e) { t.skip('no g++'); return; }
  const r = defaultRobot(); r.motor.v_min_mps = 0.02; r.pid.offset_mm = 0;
  const g = { kp: 1.7, ki: 0.4, kd: 0.25, speed: 0.8 };
  const dir = mkdtempSync(join(tmpdir(), 'rt-pid-'));
  try {
    writeFileSync(join(dir, 'pid_gains.h'), cFunction(r, g));
    writeFileSync(join(dir, 'main.cpp'), `#include "${root}sim/rt_api.cpp"
#include "pid_gains.h"
#include <cstdio>
int main() {
  Config c; c.n_sensors = 3; c.sensors[0].angle_deg = -45; c.sensors[1].angle_deg = 0; c.sensors[2].angle_deg = 45;
  c.kp = PID_KP; c.ki = PID_KI; c.kd = PID_KD; c.pid_error_clip_m = PID_ERROR_CLIP_M; c.pid_integral_limit = PID_INTEGRAL_LIMIT;
  c.pid_d_filter_alpha = PID_D_FILTER_ALPHA; c.pid_speed_mode = 1; c.pid_speed_frac = PID_SPEED_FRAC; c.pid_v_min = PID_V_MIN_MPS;
  c.max_linear_speed_mps = PID_V_MAX_MPS;
  Pid p; p.Configure(c); p.Reset();
  pid_state_t s; pid_reset(&s);
  unsigned x = 12345; int bad = 0; float worst = 0;
  for (int k = 0; k < 5000; ++k) {
    x = x * 1664525u + 1013904223u; float l = 0.05f + (x >> 8) / 16777216.0f * 0.4f;
    x = x * 1664525u + 1013904223u; float r = 0.05f + (x >> 8) / 16777216.0f * 0.4f;
    float dt = (k % 3 == 0) ? 0.025f : 0.031f;
    float d[3] = {r, 0.3f, l}, el, er, cl, cr;
    p.Control(d, dt, el, er); pid_control(&s, l, r, dt, &cl, &cr);
    float e = std::max(std::fabs(el - cl), std::fabs(er - cr)); worst = std::max(worst, e); if (e > 1e-5f) ++bad;
  }
  std::printf("%d %g\\n", bad, worst);
  return 0;
}
`);
    execFileSync('g++', ['-std=c++17', '-O2', '-ffp-contract=off', '-w', join(dir, 'main.cpp'), '-o', join(dir, 'a.out')]);
    const [bad, worst] = execFileSync(join(dir, 'a.out')).toString().trim().split(' ').map(Number);
    assert.equal(bad, 0, `largest difference ${worst} m/s`);
  } finally { rmSync(dir, { recursive: true, force: true }); }
  // the config defines and the model file carry the same cruise speed
  const h = configH(r, g);
  assert.match(h, /DEF_PID_SPEED_FRAC\s+0\.800000f/); assert.match(h, /DEF_PID_V_MIN\s+0\.0200000f/); assert.match(h, /DEF_PID_SLOWDOWN_FRAC\s+0\.00000f/);
  assert.match(modelTxt(r, g), /\nconst_speed 1\n/);   // upstream PidController.hpp Load: constant-speed mode on
  assert.doesNotMatch(modelTxt(r, g), /v_min/);        // a v_min model key would switch on upstream's speed law
  assert.equal(simConfigLines(r, g), 'cs_mode 1\ncs_speed_frac 0.8\ncs_v_min_mps 0.02\n');
});

test('the generated sketch runs the same PID with speed_frac and v_min filled in', () => {
  const r = defaultRobot(); r.motor.v_min_mps = 0.021; r.pid.speed_frac = 0.7;
  const c = generateSketch(r, r.hardware).code;
  assert.ok(c.includes('const float PID_SPEED_FRAC = 0.7f;'));
  assert.ok(c.includes(`const float PID_V_MAX = ${r.motor.max_speed_mps}f, PID_V_MIN = 0.021f;`));
  assert.ok(c.includes('float base = PID_V_MIN + PID_SPEED_FRAC * (PID_V_MAX - PID_V_MIN);'));
  assert.ok(c.includes('vl = constrain(base - turn, -PID_V_MAX, PID_V_MAX);') && c.includes('vr = constrain(base + turn, -PID_V_MAX, PID_V_MAX);'));
  assert.ok(c.includes('const int PID_LEFT = 2, PID_RIGHT = 0;'));
  assert.ok(c.includes('IS("CAL VMIN")') && c.includes('STARTS("CAL RUN")') && c.includes('IS("CAL STOP")'));
  assert.ok(!/slowdown/i.test(c.slice(c.indexOf('wall-following PID'))));
});

test('CAL VMIN answer: parser and sanity check', () => {
  const line = 'CAL VMIN: left fwd 0.0210 m/s at PWM 36, right fwd 0.0190 m/s at PWM 39, left rev 0.0225 m/s at PWM 42, right rev 0.0200 m/s at PWM 39';
  const v = P.parseVmin(line);
  assert.equal(v.vmin, 0.0225); assert.equal(v.lo, 0.019); assert.equal(v.missing, 0);
  assert.deepEqual(v.wheels.right_fwd, { v: 0.019, pwm: 39 });
  const r = defaultRobot();
  assert.equal(P.sanity('vmin', v, r), null);
  assert.equal(P.sanity('vmin', P.parseVmin(line.replace('0.0225 m/s at PWM 42', 'none')), r)[1], 'chk.vmin_missing');
  assert.equal(P.sanity('vmin', { ...v, vmin: 0.2 }, r)[1], 'chk.vmin_range');       // above the top speed (0.156)
  assert.equal(P.sanity('vmin', { ...v, vmin: 0.1 }, r)[1], 'chk.vmin_high');        // more than half of it
  assert.equal(P.sanity('vmin', { ...v, vmin: 0.05, lo: 0.02 }, r)[1], 'chk.vmin_spread');
  assert.equal(P.parseVmin('CAL VMAX: nothing'), null);
});
