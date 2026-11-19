// train-core.js - the training problem, shared by the worker and the page:
// the set of corridors, the episode config and the candidate encoding. The reward is the upstream one
// (train.cpp EvaluateRobot, ported in sim/rt_engine.hpp Evaluate): metres driven along the corridor, weighted
// by how parallel and centred the robot is, minus wobble, jerky commands and crashes, plus a finish/speed bonus.
import { robotToConfig } from './store.js';
import { EndlessCorridor, mergeEnv } from './corridor.js';
import { simOptions } from './simview.js';

export const TRACK_LEN = 4.0;  // m of each training corridor

// Typical wall errors of a hand-built corridor, used for the "type a width" training set.
export const TRAIN_ERRORS = {
  jitter: { on: true, mm: 2 }, tilt: { on: true, deg: 1.0 }, bumps: { on: true, mm: 8, per_m: 0.3 },
  gaps: { on: true, mm: 60, per_m: 0.15 }, steps: { on: true, mm: 8, per_m: 0.2 },
};

/** Settings of the training set. mode 'width': straight corridors around a typed width with the typical wall errors;
 *  mode 'mine': straight corridors with the wall errors and noise set on the Corridor page. */
export function defaultTrainSettings() { return { mode: 'width', width_mm: 200, spread_pct: 15, angle_deg: 30 }; }

/** The environment the trainer uses (always straight corridors). */
export function trainEnv(settings, env) {
  const base = mergeEnv(env);
  base.mode = 'endless';
  if (settings.mode === 'width') { base.width_mm = settings.width_mm; base.errors = JSON.parse(JSON.stringify(TRAIN_ERRORS)); }
  base.start = { angle_deg: settings.angle_deg, offset_mm: 0, random: true };
  base.spread_pct = settings.spread_pct;
  return base;
}

/** The training set: 6 straight corridors, widths spread over +-spread_pct around the width, each with its own
 *  wall errors (different seeds). Every episode after the first starts at a random angle and position. */
export function trackSet(env) {
  const W = env.width_mm, sp = (env.spread_pct ?? 15) / 100;
  const corr = (w, seed) => {
    const e = mergeEnv({ ...env, width_mm: Math.round(w), seed });
    const c = new EndlessCorridor(e); c.ensure(TRACK_LEN + 1);
    const L = TRACK_LEN, hw = c.halfWidthAt(L - 0.2);
    return { walls: c.wallsFlat(-0.5, L + 0.3), center: c.centerFlat(-0.5, L + 0.3), spawn: [0.3, 0, 0], finish: [L - 0.2, hw + 0.05, L - 0.2, -hw - 0.05], width: Math.round(w) };
  };
  return [-1, -0.5, 0, 0, 0.5, 1].map((f, i) => corr(W * (1 + f * sp), 101 * (i + 1)));
}

/** engine config for a candidate {kp, ki, kd, speed (fraction of top speed)} at a noise multiplier */
export function episodeConfig(robot, env, cand, noise = 1) {
  const o = simOptions(env); o.noise = noise;
  const c = robotToConfig(robot, o);
  const speed = robot.motor.max_speed_mps * cand.speed;
  return {
    ...c, kp: cand.kp, ki: cand.ki, kd: cand.kd, base_speed_frac: cand.speed,
    max_episode_s: Math.min(240, 3 * TRACK_LEN / Math.max(0.02, speed) + 5),
    spawn_heading_random_deg: Math.max(5, Math.abs(env.start.angle_deg)), spawn_lateral_random_frac: 0.5, spawn_margin_m: 0.01,
    parallel_weight: 4, center_weight: 10, heading_sigma_deg: 12, center_sigma_frac: 0.3, crash_penalty: 200,
    wobble_weight: 60, smooth_weight: 20, speed_bonus: 800, finish_bonus: 500,
  };
}

export function rngJS(seed) { let a = seed >>> 0; return () => { a = (a + 0x6D2B79F5) >>> 0; let t = a; t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; }; }
function gaussJS(r) { let u = 0; while (u === 0) u = r(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * r()); }

/** Grid of starting candidates scaled to the robot: Kp ~ top speed / half corridor width. */
export function grid(robot, env, trainSpeed) {
  const k0 = robot.motor.max_speed_mps / (env.width_mm / 2000);
  const kps = [0.05, 0.1, 0.2, 0.4, 0.8, 1.6].map((m) => k0 * m);
  const kdr = [0, 0.1, 0.25, 0.5, 1.0];  // Kd / Kp (seconds)
  const kir = [0, 0.05];                  // Ki / Kp (1/s)
  const sp = trainSpeed ? [0.6, 0.8, 1.0] : [robot.pid.speed_mps / robot.motor.max_speed_mps];
  const out = [];
  for (const kp of kps) for (const d of kdr) for (const i of kir) for (const s of sp) out.push({ kp, kd: kp * d, ki: kp * i, speed: Math.min(1, s) });
  return out;
}
/** upstream PidBrain::Mutate style: each gain x exp(N(0, 0.25)) with probability 1/2, at least one; speed + N(0, 0.05) */
export function mutate(c, r, trainSpeed) {
  const n = { ...c }, keys = ['kp', 'ki', 'kd'].concat(trainSpeed ? ['speed'] : []);
  const forced = keys[Math.floor(r() * keys.length)];
  for (const k of keys) if (k === forced || r() < 0.5) {
    if (k === 'speed') n.speed = Math.min(1, Math.max(0.2, n.speed + 0.05 * gaussJS(r)));
    else n[k] = Math.min(50, Math.max(1e-4, Math.max(n[k], k === 'ki' ? 1e-3 : 1e-4) * Math.exp(0.25 * gaussJS(r))));
  }
  return n;
}
