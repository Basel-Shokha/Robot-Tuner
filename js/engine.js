// engine.js - JavaScript wrapper of the WebAssembly engine (sim/rt_api.cpp). Works in pages, workers and Node.
import createRtSim from '../wasm/rt_sim.mjs';

// Layout of rt_state() (keep in sync with sim/rt_api.cpp).
export const S = {
  t: 0, x: 1, y: 2, theta: 3, v_l: 4, v_r: 5, alive: 6, finished: 7, state: 8, enc_l: 9, enc_r: 10,
  wheel_l: 11, wheel_r: 12, eff_l: 13, eff_r: 14, err: 15, p: 16, i: 17, d: 18, turn: 19, base: 20, sat: 21,
  pwm_l: 22, pwm_r: 23, enc_dist_l: 24, enc_dist_r: 25, s: 26, center_err: 27, heading_err: 28, half_width: 29,
  guide_ok: 30, left_idx: 31, right_idx: 32, dists: 33, raw: 38, steps: 43, ticks_l: 44, ticks_r: 45,
  emerg_count: 46, wscale_l: 47, wscale_r: 48, mismatch_l: 49, mismatch_r: 50, angle_bias: 51,
};
export const STATE_LEN = 64;
export const MAXS = 5;

let modPromise = null;
export function loadEngine() {
  if (!modPromise) modPromise = createRtSim().then((m) => new Engine(m));
  return modPromise;
}

export class Engine {
  constructor(m) { this.m = m; }
  create() { return new Sim(this, this.m._rt_create()); }
  // copy a float array into module memory; returns pointer (caller frees)
  _floats(arr) {
    const n = arr.length;
    const p = this.m._malloc(Math.max(4, n * 4));
    this.m.HEAPF32.set(arr instanceof Float32Array ? arr : Float32Array.from(arr), p >> 2);
    return p;
  }
  _str(s) {
    const n = this.m.lengthBytesUTF8(s) + 1;
    const p = this.m._malloc(n);
    this.m.stringToUTF8(s, p, n);
    return p;
  }
  _read(ptr, n) { return this.m.HEAPF32.slice(ptr >> 2, (ptr >> 2) + n); }
}

export class Sim {
  constructor(engine, id) { this.e = engine; this.m = engine.m; this.id = id; this.nSensors = 3; this.nWheels = 2; }
  destroy() { this.m._rt_destroy(this.id); }
  /** "key value" lines, or an object {key: value | [values]}. Returns the number of unknown keys. */
  config(cfg) {
    const text = typeof cfg === 'string' ? cfg : configText(cfg);
    const mN = /(^|\n)\s*n_sensors\s+(\d+)/.exec(text); if (mN) this.nSensors = +mN[2];
    const mW = /(^|\n)\s*four_wheel\s+(\d+)/.exec(text); if (mW) this.nWheels = +mW[2] ? 4 : 2;
    const p = this.e._str(text);
    const u = this.m._rt_config(this.id, p);
    this.m._free(p);
    return u;
  }
  configReset() { this.m._rt_config_reset(this.id); }
  /** walls: flat [x1,y1,x2,y2, ...] */
  walls(flat) { const p = this.e._floats(flat); this.m._rt_walls_set(this.id, p, flat.length >> 2); this.m._free(p); }
  /** centre line: flat [x, y, half_width, ...] */
  centerline(flat) { const p = this.e._floats(flat); this.m._rt_centerline_set(this.id, p, (flat.length / 3) | 0); this.m._free(p); }
  finish(on, x1 = 0, y1 = 0, x2 = 0, y2 = 0) { this.m._rt_finish_set(this.id, on ? 1 : 0, x1, y1, x2, y2); }
  spawn(x, y, h) { this.m._rt_spawn_set(this.id, x, y, h); }
  randomSpawn(seed) { const p = this.m._rt_random_spawn(this.id, seed >>> 0); const a = this.e._read(p, 3); return { x: a[0], y: a[1], h: a[2] }; }
  reset(x, y, h, seed = 1, sampleBias = true) { this.m._rt_reset(this.id, x, y, h, seed >>> 0, sampleBias ? 1 : 0); }
  external(on) { this.m._rt_set_external(this.id, on ? 1 : 0); }
  cmd(vl, vr) { this.m._rt_ext_cmd(this.id, vl, vr); }
  step(n = 1) { return this.m._rt_step(this.id, n) === 1; }
  stateRaw() { return this.e._read(this.m._rt_state(this.id), STATE_LEN); }
  state() {
    const a = this.stateRaw();
    const o = {};
    for (const [k, i] of Object.entries(S)) o[k] = a[i];
    o.dists = Array.from(a.subarray(S.dists, S.dists + this.nSensors));
    o.raw = Array.from(a.subarray(S.raw, S.raw + this.nSensors));
    o.angle_bias = Array.from(a.subarray(S.angle_bias, S.angle_bias + this.nSensors));
    return o;
  }
  /** per sensor {x, y, a, d, half, max} */
  rays() {
    const a = this.e._read(this.m._rt_rays(this.id), 6 * this.nSensors);
    const out = [];
    for (let i = 0; i < this.nSensors; i++) out.push({ x: a[6 * i], y: a[6 * i + 1], a: a[6 * i + 2], d: a[6 * i + 3], half: a[6 * i + 4], max: a[6 * i + 5] });
    return out;
  }
  /** [[x,y]x4] for the body then each wheel */
  footprint() {
    const n = 1 + this.nWheels;
    const a = this.e._read(this.m._rt_footprint(this.id), 8 * n);
    const polys = [];
    for (let k = 0; k < n; k++) { const q = []; for (let j = 0; j < 4; j++) q.push([a[8 * k + 2 * j], a[8 * k + 2 * j + 1]]); polys.push(q); }
    return polys;
  }
  evaluate(seed, generation, episode) {
    const a = this.e._read(this.m._rt_evaluate(this.id, seed >>> 0, generation, episode), 9);
    return { score: a[0], crashed: a[1] > 0, finished: a[2] > 0, steps: a[3], forward_m: a[4], meanCenter: a[5], rmsCenter: a[6], meanHeading: a[7], time: a[8], raw: a };
  }
}

/** {key: value|[v,...]} -> "key v ...\n" text, numbers written exactly (shortest round-trip form). */
export function configText(obj) {
  let s = '';
  for (const [k, v] of Object.entries(obj)) {
    if (v === undefined || v === null) continue;
    s += k + ' ' + (Array.isArray(v) ? v.join(' ') : (typeof v === 'boolean' ? (v ? 1 : 0) : v)) + '\n';
  }
  return s;
}
