// tutorial-sim.js - the small robot model behind the tutorial demos (pure functions, no DOM; tested in
// tests/js/tutorial.test.mjs). The full simulator is in the playground.
//   y: distance of the robot from the corridor centre (m, + = left), th: heading (rad, + = left)
//   two side sensors at +-45 deg see the walls; e = d_left - d_right ~ -(2.83 y + 0.4 th) (small angles)
//   u = Kp e + Ki int(e) + Kd de/dt (m/s); v_left = base - u, v_right = base + u; omega = 2u / track; y' = v sin th
import { gauss, rng } from './corridor.js';

/** p: {kp, ki, kd, delay (s), noise (m), mismatch, bias (m), y0 (m), th0, v (m/s), W (corridor width, m),
 *      sensor_deg (side sensors at +-45 (default) or +-90 deg), crash (stop at the wall, default true), alpha (D filter, 0..1), ilimit} */
export function simulate(p, T = 8, dt = 0.025) {
  const v = p.v ?? 0.15, track = 0.125, W = p.W ?? 0.25, alpha = p.alpha ?? 0.3, ilim = p.ilimit ?? 0.5;
  let y = p.y0 ?? 0.04, th = p.th0 ?? 0, integ = 0, prev = null, dfil = 0;
  const r = rng(7), delaySteps = Math.round((p.delay ?? 0) / dt), hist = [];
  const out = { t: [], x: [], y: [], e: [], turn: [], crash: false };
  let x = 0;
  for (let k = 0; k * dt <= T; k++) {
    const sa = (p.sensor_deg ?? 45) * Math.PI / 180;              // sensor angle from straight ahead
    const cl = Math.sin(sa - th), cr = Math.sin(sa + th);
    const dl = cl > 0.1 ? Math.min(1.3, (W / 2 - y) / cl) : 1.3, dr = cr > 0.1 ? Math.min(1.3, (W / 2 + y) / cr) : 1.3;
    let e = dl - dr + (p.noise ?? 0) * gauss(r) + (p.bias ?? 0);
    hist.push(e); e = hist[Math.max(0, hist.length - 1 - delaySteps)];
    integ = Math.max(-ilim, Math.min(ilim, integ + e * dt));            // anti-windup: clamp the integral
    const raw = prev === null ? 0 : (e - prev) / dt; prev = e;
    dfil = alpha * raw + (1 - alpha) * dfil;                            // low-pass filter on D
    let turn = (p.kp ?? 0) * e + (p.ki ?? 0) * integ + (p.kd ?? 0) * dfil;
    turn = Math.max(-v, Math.min(v, turn));                             // a wheel cannot go below 0 here
    const om = 2 * turn / track + (p.mismatch ?? 0) * v / track;
    th += om * dt; y += v * Math.sin(th) * dt; x += v * Math.cos(th) * dt;
    out.t.push(k * dt); out.x.push(x * 100); out.y.push(y * 1000); out.e.push(e * 1000); out.turn.push(turn);
    if ((p.crash ?? true) && Math.abs(y) > W / 2 - 0.035) { out.crash = true; break; }
  }
  return out;
}

/** The tutorial's main graph: distance from the centre (cm) against distance driven (cm), start 5 cm off centre.
 *  A wide corridor (no crash) and a small sensing delay, like a real robot. */
export const GRAPH = { sensor_deg: 90, y0: 0.05, v: 0.15, W: 0.6, delay: 0.06, crash: false, length_cm: 100 };
export const GRAPH_PRESETS = {
  good: { kp: 0.4, ki: 0, kd: 0.4 },
  kp_high: { kp: 3, ki: 0, kd: 0.4 },
  p_only: { kp: 0.5, ki: 0, kd: 0 },
  kd_high: { kp: 0.4, ki: 0, kd: 2.5 },
};
export function corridorCurve(g) {
  const T = GRAPH.length_cm / 100 / GRAPH.v + 0.2;
  const s = simulate({ ...GRAPH, ...g }, T);
  const xs = [], ys = [];
  for (let i = 0; i < s.x.length && s.x[i] <= GRAPH.length_cm; i++) { xs.push(s.x[i]); ys.push(s.y[i] / 10); }
  return { x: xs, y: ys };
}
/** distance driven (cm) after which the robot stays within `band` cm of the centre (Infinity if it never does) */
export function settleDistance(c, band = 0.5) {
  let last = -1;
  for (let i = 0; i < c.y.length; i++) if (Math.abs(c.y[i]) > band) last = i;
  return last === c.y.length - 1 ? Infinity : last < 0 ? 0 : c.x[last + 1];
}
/** wheel commands from the PID output: base - u / base + u, base = v_min + speed_frac (v_max - v_min), clipped */
export function wheelCommands(u, { vmin, vmax, frac }) {
  const base = vmin + frac * (vmax - vmin);
  const clip = (v) => Math.max(vmin, Math.min(vmax, v));
  const left = base - u, right = base + u;
  return { base, left, right, leftClip: clip(left), rightClip: clip(right), clipped: clip(left) !== left || clip(right) !== right };
}
