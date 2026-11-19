// simview.js - shared by the corridor page, the playground and the training preview:
//   Scene   the environment (endless corridor or a drawn/imported track), its walls and centre line
//   Runner  one simulated robot (one engine instance) with its history for plots
//   drawScene()  draws walls, centre line, paths, robots, sensor rays; follows the lead robot
import { robotToConfig } from './store.js';
import { EndlessCorridor, buildTrack, TRACK_PRESETS, feasibleStart, rng } from './corridor.js';
import { S } from './engine.js';
import { css, PALETTE } from './draw.js';

export function simOptions(env) {
  const n = env.noise;
  return { noise: n.global, sensorNoise: n.sensor, encoderNoise: n.encoder, mismatch: n.mismatch_pct / 100, delayMs: n.delay_ms, angleBias: n.angle_bias ? 2 : 0 };
}

export class Scene {
  constructor(env) {
    this.env = env;
    this.endless = env.mode !== 'track';
    if (this.endless) { this.corr = new EndlessCorridor(env); this.win = [-1, -1]; }
    else {
      const tr = env.track;
      if (tr.imported) this.track = tr.imported;
      else this.track = buildTrack(tr.points || TRACK_PRESETS[tr.preset] || TRACK_PRESETS.straight, tr.width_mm / 1000, tr.radius_mm / 1000,
        env.errors.jitter.on ? env.errors.jitter.mm / 1000 : 0, env.seed, !!tr.closed);
    }
  }
  /** start pose {x, y, h, angle, offset, note} for the env's start settings (seed used when random) */
  start(robot, seed = 1) {
    const st = this.env.start;
    let angle = st.angle_deg, offset = st.offset_mm / 1000;
    let x0, y0, h0, hw;
    if (this.endless) { x0 = 0.3; y0 = 0; h0 = 0; hw = this.corr.halfWidthAt(0.3); }
    else { [x0, y0, h0] = this.track.spawn; hw = this.track.center.length ? this.track.center[2] : 0.1; }
    if (st.random) {
      const r = rng(seed * 2654435761 >>> 0);
      angle = (2 * r() - 1) * st.angle_deg;
      const f = feasibleStart(robot, hw, angle, 0);
      angle = f.angle;
      offset = f.range[0] + r() * (f.range[1] - f.range[0]);
    }
    const f = feasibleStart(robot, hw, angle, offset);
    const nx = -Math.sin(h0), ny = Math.cos(h0);
    return { x: x0 + nx * f.offset, y: y0 + ny * f.offset, h: h0 + f.angle * Math.PI / 180, angle: f.angle, offset: f.offset, note: f.note };
  }
  /** keep the engine's walls around the robots (endless corridor). Returns true when they changed. */
  update(runners, force = false) {
    if (!this.endless) {
      if (force) for (const r of runners) this.applyTo(r.sim);
      return force;
    }
    let lo = Infinity, hi = -Infinity;
    for (const r of runners) { const x = r.last ? r.last[S.x] : 0; lo = Math.min(lo, x); hi = Math.max(hi, x); }
    if (!force && lo - 1.5 >= this.win[0] && hi + 3 <= this.win[1]) return false;
    this.win = [Math.floor(lo) - 2, Math.ceil(hi) + 6];
    this.corr.ensure(this.win[1] + 1);
    for (const r of runners) this.applyTo(r.sim);
    return true;
  }
  applyTo(sim) {
    if (this.endless) { sim.walls(this.corr.wallsFlat(this.win[0], this.win[1])); sim.centerline(this.corr.centerFlat(this.win[0], this.win[1])); sim.finish(false); }
    else {
      sim.walls(this.track.walls);
      if (this.track.center.length) sim.centerline(this.track.center);
      if (this.track.finish) sim.finish(true, ...this.track.finish); else sim.finish(false);
    }
  }
  get loopLength() { return !this.endless && this.track.closed ? this.track.length : 0; }
  wallsInView(x0, x1) { return this.endless ? this.corr.wallsFlat(x0, x1) : this.track.walls; }
  centerInView(x0, x1) { return this.endless ? this.corr.centerFlat(x0, x1) : this.track.center; }
}

const HIST_KEYS = ['t', 'x', 'y', 'ce', 'he', 'err', 'vl', 'vr', 'wl', 'wr', 'p', 'i', 'd', 'sat'];
// laps on a closed track: the guide position s jumps back by more than half the loop at the start line
export class Runner {
  /** engine: Engine; robot: robot JSON (already patched with the gains to use); env; label; color */
  constructor(engine, robot, env, opts = {}) {
    this.sim = engine.create();
    this.robot = robot; this.env = env; this.label = opts.label || ''; this.color = opts.color || PALETTE[0];
    this.configure(robot);
    this.clear();
  }
  configure(robot, extra = {}) {
    this.robot = robot;
    const c = { ...robotToConfig(robot, simOptions(this.env)), ...extra };
    this.dt = c.dt;
    this.sim.config(c);
  }
  clear() { this.h = {}; for (const k of HIST_KEYS) this.h[k] = []; this.last = null; this.crashT = null; this.finishedT = null; this.laps = 0; this.prevS = null; this.loopLen = 0; }
  reset(pose, seed, loopLen = 0) { this.clear(); this.loopLen = loopLen; this.sim.reset(pose.x, pose.y, pose.h, seed, true); this.last = this.sim.stateRaw(); this.record(); }
  get alive() { return !this.last || (this.last[S.alive] > 0 && !(this.last[S.finished] > 0)); }
  step(n = 1) {
    for (let k = 0; k < n; k++) {
      if (!this.alive) break;
      this.sim.step(1);
      this.last = this.sim.stateRaw();
      this.record();
      if (this.last[S.alive] === 0) this.crashT = this.last[S.t];
      if (this.last[S.finished] > 0) this.finishedT = this.last[S.t];
    }
  }
  record() {
    const s = this.last, h = this.h;
    if (this.loopLen > 0 && s[S.guide_ok]) {
      if (this.prevS !== null && this.prevS - s[S.s] > this.loopLen / 2) this.laps++;
      if (this.prevS !== null && s[S.s] - this.prevS > this.loopLen / 2) this.laps--;  // drove backwards over the line
      this.prevS = s[S.s];
    }
    h.t.push(s[S.t]); h.x.push(s[S.x]); h.y.push(s[S.y]);
    h.ce.push(s[S.guide_ok] ? s[S.center_err] * 1000 : NaN); h.he.push(s[S.guide_ok] ? s[S.heading_err] * 180 / Math.PI : NaN);
    h.err.push(s[S.err] * 1000); h.vl.push(s[S.v_l]); h.vr.push(s[S.v_r]); h.wl.push(s[S.eff_l]); h.wr.push(s[S.eff_r]);
    h.p.push(s[S.p]); h.i.push(s[S.i]); h.d.push(s[S.d]); h.sat.push(s[S.sat]);
  }
  /** summary of the run so far: mean |centre error| (mm), max, mean |heading| (deg), distance (m) */
  stats(fromT = 0) {
    let n = 0, sc = 0, mx = 0, sh = 0;
    for (let k = 0; k < this.h.t.length; k++) {
      if (this.h.t[k] < fromT || Number.isNaN(this.h.ce[k])) continue;
      n++; sc += Math.abs(this.h.ce[k]); mx = Math.max(mx, Math.abs(this.h.ce[k])); sh += Math.abs(this.h.he[k]);
    }
    let dist = 0; for (let k = 1; k < this.h.x.length; k++) dist += Math.hypot(this.h.x[k] - this.h.x[k - 1], this.h.y[k] - this.h.y[k - 1]);
    return { meanCe: n ? sc / n : NaN, maxCe: mx, meanHe: n ? sh / n : NaN, dist, t: this.last ? this.last[S.t] : 0, crashed: this.crashT !== null, finished: this.finishedT !== null, laps: this.laps };
  }
  destroy() { this.sim.destroy(); }
}

/** Draw the scene; runners[0] is the lead (rays + full colour), the others are ghosts. */
export function drawScene(view, scene, runners, opts = {}) {
  const lead = runners[0];
  if (opts.follow !== false && lead && lead.last && !view.userMoved) {
    if (scene.endless) {
      const W = scene.env.width_mm / 1000;
      view.scale = Math.min(view.h / (W * 2.6), view.w / 1.6);
      view.cx = lead.last[S.x] + view.w / view.scale * 0.25; view.cy = 0;
    } else {
      const [a, b, c, d] = scene.track.bounds;
      view.fit(a, b, c, d, 0.04);
    }
  }
  const x0 = view.wx(0) - 0.5, x1 = view.wx(view.w) + 0.5;
  view.begin();
  view.grid(0.1);
  const cf = scene.centerInView(x0, x1), cl = [];
  for (let i = 0; i < cf.length; i += 3) cl.push([cf[i], cf[i + 1]]);
  view.polyline(cl, css('--muted'), 1, [6, 6], 0.7);
  view.walls(scene.wallsInView(x0, x1));
  if (!scene.endless && scene.track.finish) { const f = scene.track.finish; view.polyline([[f[0], f[1]], [f[2], f[3]]], css('--ok'), 3, [4, 4]); }
  runners.forEach((r, i) => {
    const pts = []; for (let k = 0; k < r.h.x.length; k += 1) pts.push([r.h.x[k], r.h.y[k]]);
    view.polyline(pts, r.color, i === 0 ? 2.2 : 1.6, null, i === 0 ? 1 : 0.85);
  });
  runners.forEach((r, i) => {
    if (!r.last) return;
    if (i === 0 && opts.rays !== false) view.rays(r.sim.rays());
    view.robot(r.sim.footprint(), r.color, i === 0 ? 1 : 0.6, r.last[S.theta]);
    if (r.crashT !== null) view.text(r.last[S.x], r.last[S.y] + 0.06, '✕ ' + (opts.crashLabel || 'crash'), css('--bad'), 'center');
  });
  view.scaleBar();
}
