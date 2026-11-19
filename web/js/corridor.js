// corridor.js - environments for the corridor simulator:
//   * EndlessCorridor: a straight corridor along +x, generated chunk by chunk (deterministic per seed), with
//     the construction errors of a real hand-built corridor, each switchable with its own size:
//       jitter (small random wall unevenness), tilts (boards slightly rotated), bumps (things sticking out),
//       gaps (missing wall pieces, e.g. between boards), width steps (one wall jumps in or out).
//   * Track: a drawn centre line (corners and S-bends rounded with a radius), offset to two walls.
//   * startPose(): the start pose (angle, offset) clipped so the robot never starts inside a wall.
// Walls are flat arrays [x1, y1, x2, y2, ...] in metres; centre lines are [x, y, half_width, ...].

export function rng(seed) {  // mulberry32
  let a = seed >>> 0;
  return () => { a = (a + 0x6D2B79F5) >>> 0; let t = a; t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
}
export function gauss(r) { let u = 0, v = 0; while (u === 0) u = r(); v = r(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v); }

export function defaultEnv() {
  return {
    mode: 'endless', width_mm: 200, seed: 1,
    errors: {
      jitter: { on: true, mm: 2 }, tilt: { on: true, deg: 1.0 }, bumps: { on: false, mm: 10, per_m: 0.4 },
      gaps: { on: false, mm: 80, per_m: 0.3 }, steps: { on: false, mm: 15, per_m: 0.25 },
    },
    track: { preset: 's_bend', points: null, width_mm: 250, radius_mm: 300, closed: false },
    start: { angle_deg: 20, offset_mm: 0, random: false },
    noise: { global: 1, sensor: 1, encoder: 1, mismatch_pct: 0, delay_ms: 0, angle_bias: true },
    duration_s: 40,
  };
}
export function loadEnv() {
  try { const s = localStorage.getItem('rt.env'); if (s) return mergeEnv(JSON.parse(s)); } catch (e) { /* ignore */ }
  return defaultEnv();
}
export function saveEnv(env) { try { localStorage.setItem('rt.env', JSON.stringify(env)); } catch (e) { /* ignore */ } }
export function mergeEnv(e) {
  const d = defaultEnv();
  const out = { ...d, ...e, errors: { ...d.errors }, track: { ...d.track, ...(e.track || {}) }, start: { ...d.start, ...(e.start || {}) }, noise: { ...d.noise, ...(e.noise || {}) } };
  for (const k of Object.keys(d.errors)) out.errors[k] = { ...d.errors[k], ...((e.errors || {})[k] || {}) };
  return out;
}

// ------------------------------------------------------------------------------------------- endless
const CHUNK = 2.0;   // m per generated chunk
const BOARD = [0.4, 0.8];  // board length range (m): the unit a wall is built from
export class EndlessCorridor {
  constructor(env) {
    this.env = env; this.W = env.width_mm / 1000; this.e = env.errors;
    this.walls = []; this.center = []; this.until = -0.5; this.chunk = 0;
    // per side state: offset of the wall from its nominal line (m, + = into the corridor), nominal shift from steps
    this.side = [0, 1].map(() => ({ off: 0, step: 0, prev: null }));
    this.ensure(4);
  }
  /** generate until x >= upto */
  ensure(upto) { while (this.until < upto) this._chunk(); }
  _chunk() {
    const r = rng((this.env.seed * 7919 + this.chunk * 104729) >>> 0);
    const x0 = this.until, x1 = x0 + CHUNK;
    const clean = x0 < 0.6;  // keep the start area simple (only jitter)
    for (let s = 0; s < 2; s++) {
      const st = this.side[s], sign = s === 0 ? 1 : -1;  // 0 = left wall (+y), 1 = right wall (-y)
      let x = x0;
      while (x < x1 - 1e-9) {
        const L = Math.min(x1 - x, BOARD[0] + r() * (BOARD[1] - BOARD[0]));
        // width step at the start of this board
        let jumped = false;
        if (!clean && this.e.steps.on && r() < this.e.steps.per_m * L) {
          const d = (r() < 0.5 ? -1 : 1) * this.e.steps.mm / 1000;
          const nstep = Math.max(-2 * this.e.steps.mm / 1000, Math.min(2 * this.e.steps.mm / 1000, st.step + d));
          if (nstep !== st.step) { this._pt(s, x, st, sign); st.step = nstep; jumped = true; }
        }
        // tilt: the board's end offset (mean-reverting so walls do not drift away)
        let endOff = st.off;
        if (!clean && this.e.tilt.on) {
          const a = gauss(r) * this.e.tilt.deg * Math.PI / 180;
          const tmax = (this.e.tilt.max_mm ?? 20) / 1000;  // how far a tilted board may lean in / out
          endOff = Math.max(-tmax, Math.min(tmax, st.off * 0.4 + Math.tan(a) * L));
        } else endOff = 0;
        // sample points along the board every ~10 cm (jitter), with bumps / gaps
        const n = Math.max(1, Math.round(L / 0.1));
        for (let k = 0; k <= n; k++) {
          if (k === 0 && !jumped && st.prev && Math.abs(st.prev[0] - x) < 1e-9) continue;
          const xx = x + (L * k) / n;
          const o = st.off + (endOff - st.off) * (k / n) + (this.e.jitter.on ? gauss(r) * this.e.jitter.mm / 1000 / 2 : 0);
          this._pt(s, xx, { ...st, off: o }, sign);
          if (k < n && !clean) {
            const seg = L / n;
            if (this.e.bumps.on && r() < this.e.bumps.per_m * seg) {
              const bl = Math.min(seg * 0.9, 0.03 + r() * 0.05), h = this.e.bumps.mm / 1000 * (0.6 + 0.4 * r()), bx = xx + r() * Math.max(0, seg - bl);
              this._pt(s, bx, { ...st, off: o }, sign);
              this._pt(s, bx + bl * 0.3, { ...st, off: o + h }, sign);
              this._pt(s, bx + bl * 0.7, { ...st, off: o + h }, sign);
              this._pt(s, bx + bl, { ...st, off: o }, sign);
            }
            if (this.e.gaps.on && r() < this.e.gaps.per_m * seg) {
              const gl = Math.min(seg * 0.9, this.e.gaps.mm / 1000 * (0.6 + 0.8 * r())), gx = xx + r() * Math.max(0, seg - gl);
              this._pt(s, gx, { ...st, off: o }, sign);
              st.prev = null;  // gap: no wall until the next point
              this._pt(s, gx + gl, { ...st, off: o }, sign);
            }
          }
        }
        st.off = endOff;
        x += L;
      }
    }
    // centre line: nominal walls including width steps (not the small errors)
    for (let x = x0; x < x1 - 1e-9; x += 0.1) {
      const yl = this.W / 2 + this.side[0].step, yr = -this.W / 2 - this.side[1].step;
      this.center.push([x, (yl + yr) / 2, (yl - yr) / 2]);
    }
    this.until = x1; this.chunk++;
  }
  _pt(s, x, st, sign) {
    // wall line: nominal +-W/2, moved outward by step, and toward the centre by off
    const y = sign * (this.W / 2 + st.step - st.off);
    const p = [x, y];
    if (st.prev) this.walls.push([st.prev[0], st.prev[1], p[0], p[1]]);
    this.side[sign > 0 ? 0 : 1].prev = p;
  }
  /** walls / centre line between xmin and xmax, flat */
  wallsFlat(xmin, xmax) {
    const out = [];
    for (const w of this.walls) if (Math.max(w[0], w[2]) >= xmin && Math.min(w[0], w[2]) <= xmax) out.push(w[0], w[1], w[2], w[3]);
    return out;
  }
  centerFlat(xmin, xmax) {
    const out = [];
    for (const c of this.center) if (c[0] >= xmin && c[0] <= xmax) out.push(c[0], c[1], c[2]);
    return out;
  }
  halfWidthAt(x) { let best = this.center[0]; for (const c of this.center) { if (c[0] > x) break; best = c; } return best ? best[2] : this.W / 2; }
}

// --------------------------------------------------------------------------------------------- tracks
export const TRACK_PRESETS = {
  straight: [[0, 0], [4, 0]],
  corner_left: [[0, 0], [1.5, 0], [1.5, 1.5]],
  corner_right: [[0, 0], [1.5, 0], [1.5, -1.5]],
  s_bend: [[0, 0], [1.0, 0], [1.6, 0.6], [2.6, 0.6], [3.2, 0], [4.2, 0]],
  zigzag: [[0, 0], [1, 0], [1.6, 0.35], [2.2, 0], [2.8, 0.35], [3.4, 0], [4.4, 0]],
  u_turn: [[0, 0], [1.6, 0], [1.6, 0.8], [0.2, 0.8]],
  // closed loops: the robot drives laps
  loop_oval: [[0, 0], [2.4, 0], [2.4, 1.2], [0, 1.2]],
  loop_square: [[0, 0], [1.6, 0], [1.6, 1.6], [0, 1.6]],
  loop_kidney: [[0, 0], [2.6, 0], [2.6, 1.4], [1.7, 1.4], [1.3, 0.8], [0.9, 1.4], [0, 1.4]],
};
export const isLoopPreset = (k) => k.startsWith('loop_');

/** Round each corner of the centre line with an arc of `radius` (smaller where segments are short). */
export function smoothPath(points, radius, closed = false) {
  const n = points.length;
  if (n < 3) return points.map((p) => [p[0], p[1]]);
  const out = closed ? [] : [[points[0][0], points[0][1]]];
  const corner = (p0, p1, p2) => {
    const a = Math.atan2(p1[1] - p0[1], p1[0] - p0[0]), b = Math.atan2(p2[1] - p1[1], p2[0] - p1[0]);
    let d = b - a; while (d > Math.PI) d -= 2 * Math.PI; while (d < -Math.PI) d += 2 * Math.PI;
    const l0 = Math.hypot(p1[0] - p0[0], p1[1] - p0[1]), l1 = Math.hypot(p2[0] - p1[0], p2[1] - p1[1]);
    if (Math.abs(d) < 1e-3) { out.push([p1[0], p1[1]]); return; }
    const tanLen = Math.min(radius * Math.tan(Math.abs(d) / 2), l0 / 2 - 1e-3, l1 / 2 - 1e-3);
    const R = tanLen / Math.tan(Math.abs(d) / 2);
    const s0 = [p1[0] - Math.cos(a) * tanLen, p1[1] - Math.sin(a) * tanLen];
    const sgn = Math.sign(d), cx = s0[0] - Math.sin(a) * R * sgn, cy = s0[1] + Math.cos(a) * R * sgn;
    const k = Math.max(2, Math.ceil(Math.abs(d) / (5 * Math.PI / 180)));
    for (let j = 0; j <= k; j++) { const th = a - sgn * Math.PI / 2 + (d * j) / k; out.push([cx + Math.cos(th) * R, cy + Math.sin(th) * R]); }
  };
  if (closed) {
    // start in the middle of the first side so the start is on a straight piece
    const P = points.concat([points[0]]);
    const mid = [(P[0][0] + P[1][0]) / 2, (P[0][1] + P[1][1]) / 2];
    out.push(mid);
    for (let i = 1; i <= n; i++) corner(P[i - 1], P[i % n === 0 ? 0 : i], points[(i + 1) % n]);
    out.push([mid[0], mid[1]]);
    return out;
  }
  for (let i = 1; i < n - 1; i++) corner(points[i - 1], points[i], points[i + 1]);
  out.push([points[n - 1][0], points[n - 1][1]]);
  return out;
}

/** Track from a centre line: {walls, center, spawn: [x, y, heading], finish: [x1, y1, x2, y2], bounds} */
export function buildTrack(points, width_m, radius_m, jitter_m = 0, seed = 1, closed = false) {
  const P = smoothPath(points, radius_m, closed);
  const hw = width_m / 2, r = rng(seed);
  const L = [], Rr = [];
  for (let i = 0; i < P.length; i++) {
    const nx1 = closed && i === P.length - 1 ? P[1] : P[i + 1], pv = closed && i === 0 ? P[P.length - 2] : P[i - 1];
    const a = nx1 ? Math.atan2(nx1[1] - P[i][1], nx1[0] - P[i][0]) : Math.atan2(P[i][1] - pv[1], P[i][0] - pv[0]);
    const a0 = pv ? Math.atan2(P[i][1] - pv[1], P[i][0] - pv[0]) : a;
    let d = a - a0; while (d > Math.PI) d -= 2 * Math.PI; while (d < -Math.PI) d += 2 * Math.PI;
    const m = a0 + d / 2, k = 1 / Math.max(0.3, Math.cos(d / 2));  // miter
    const nx = -Math.sin(m) * hw * k, ny = Math.cos(m) * hw * k;
    const j = () => (jitter_m ? gauss(r) * jitter_m / 2 : 0);
    L.push([P[i][0] + nx + j(), P[i][1] + ny + j()]); Rr.push([P[i][0] - nx + j(), P[i][1] - ny + j()]);
  }
  if (closed) { L[L.length - 1] = L[0]; Rr[Rr.length - 1] = Rr[0]; }  // the loop closes exactly
  const walls = [];
  for (const side of [L, Rr]) for (let i = 0; i + 1 < side.length; i++) walls.push(side[i][0], side[i][1], side[i + 1][0], side[i + 1][1]);
  // densify the centre line (guide) to <= 5 cm
  const center = [];
  for (let i = 0; i + 1 < P.length; i++) {
    const n = Math.max(1, Math.ceil(Math.hypot(P[i + 1][0] - P[i][0], P[i + 1][1] - P[i][1]) / 0.05));
    for (let k = 0; k < n; k++) center.push(P[i][0] + (P[i + 1][0] - P[i][0]) * k / n, P[i][1] + (P[i + 1][1] - P[i][1]) * k / n, hw);
  }
  center.push(P[P.length - 1][0], P[P.length - 1][1], hw);
  const h0 = Math.atan2(P[1][1] - P[0][1], P[1][0] - P[0][0]);
  const e = P[P.length - 1], ae = Math.atan2(e[1] - P[P.length - 2][1], e[0] - P[P.length - 2][0]);
  const fx = e[0] - Math.cos(ae) * 0.05, fy = e[1] - Math.sin(ae) * 0.05;
  const finish = closed ? null : [fx - Math.sin(ae) * hw, fy + Math.cos(ae) * hw, fx + Math.sin(ae) * hw, fy - Math.cos(ae) * hw];
  let minx = Infinity, miny = Infinity, maxx = -Infinity, maxy = -Infinity;
  for (let i = 0; i < walls.length; i += 2) { minx = Math.min(minx, walls[i]); maxx = Math.max(maxx, walls[i]); miny = Math.min(miny, walls[i + 1]); maxy = Math.max(maxy, walls[i + 1]); }
  let length = 0; for (let i = 1; i < P.length; i++) length += Math.hypot(P[i][0] - P[i - 1][0], P[i][1] - P[i - 1][1]);
  return { walls, center, spawn: [P[0][0] + Math.cos(h0) * 0.3, P[0][1] + Math.sin(h0) * 0.3, h0], finish, bounds: [minx, miny, maxx, maxy], path: P, closed, length };
}

/** Parse an upstream .track file (freeform: spawn / wall / finish / center lines). */
export function parseTrackFile(text) {
  const walls = [], center = []; let spawn = [0, 0, 0], finish = null;
  for (const line of text.split('\n')) {
    const t = line.trim().split(/\s+/);
    if (t[0] === 'wall') walls.push(+t[1], +t[2], +t[3], +t[4]);
    else if (t[0] === 'spawn') spawn = [+t[1], +t[2], +t[3] * Math.PI / 180];
    else if (t[0] === 'finish') finish = [+t[1], +t[2], +t[3], +t[4]];
    else if (t[0] === 'center') center.push(+t[1], +t[2], +t[3]);
  }
  if (!walls.length) throw new Error('no "wall" lines (only the freeform .track format is supported)');
  let minx = Infinity, miny = Infinity, maxx = -Infinity, maxy = -Infinity;
  for (let i = 0; i < walls.length; i += 2) { minx = Math.min(minx, walls[i]); maxx = Math.max(maxx, walls[i]); miny = Math.min(miny, walls[i + 1]); maxy = Math.max(maxy, walls[i + 1]); }
  return { walls, center, spawn, finish, bounds: [minx, miny, maxx, maxy] };
}

// ------------------------------------------------------------------------------------------ start pose
/** Footprint corner points of the robot in its own frame (m): body + wheels. */
export function footprintPoints(robot) {
  const b = robot.body, w = robot.wheels, pts = [];
  for (const [x, y] of [[b.front_mm, b.width_mm / 2], [b.front_mm, -b.width_mm / 2], [-b.rear_mm, b.width_mm / 2], [-b.rear_mm, -b.width_mm / 2]]) pts.push([x / 1000, y / 1000]);
  const xs = robot.drive === '4wd' ? [w.x_mm + w.wheelbase_mm / 2, w.x_mm - w.wheelbase_mm / 2] : [w.x_mm];
  for (const x of xs) for (const s of [1, -1]) for (const dx of [1, -1]) for (const dy of [1, -1])
    pts.push([(x + dx * w.diameter_mm / 2) / 1000, (s * w.track_mm / 2 + dy * w.width_mm / 2) / 1000]);
  return pts;
}
/** Lateral extent [ymin, ymax] of the footprint at heading phi (relative to the corridor). */
export function lateralExtent(pts, phi) {
  let lo = Infinity, hi = -Infinity;
  const c = Math.cos(phi), s = Math.sin(phi);
  for (const [x, y] of pts) { const yy = x * s + y * c; lo = Math.min(lo, yy); hi = Math.max(hi, yy); }
  return [lo, hi];
}
/** Clip a requested start (angle deg, offset m from the centre) so the robot fits in a corridor of half width hw. */
export function feasibleStart(robot, hw, angleDeg, offset, margin = 0.005) {
  const pts = footprintPoints(robot);
  let a = angleDeg, note = null;
  for (let tries = 0; tries < 200; tries++) {
    const [lo, hi] = lateralExtent(pts, a * Math.PI / 180);
    const ymin = -hw + margin - lo, ymax = hw - margin - hi;
    if (ymin <= ymax) {
      const y = Math.max(ymin, Math.min(ymax, offset));
      if (Math.abs(y - offset) > 1e-4) note = note || 'offset';
      return { angle: a, offset: y, note, range: [ymin, ymax] };
    }
    a -= Math.sign(a) * 1; note = 'angle';
    if (Math.abs(a) < 1) a = 0;
  }
  return { angle: 0, offset: 0, note: 'impossible', range: [0, 0] };
}
