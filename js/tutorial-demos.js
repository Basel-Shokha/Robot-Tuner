// tutorial-demos.js - small interactive demos for the PID tutorial (pure JS, a simple model of the robot
// between two walls; the full simulator is in the playground).
//   y: distance of the robot from the corridor centre (m, + = left), th: heading (rad, + = left)
//   two side sensors at +-45 deg see the walls; error = left - right ~ -(2.83 y + 0.4 th) (small angles)
//   turn = Kp e + Ki int(e) + Kd de/dt (m/s); omega = 2 turn / track; y' = v sin th
import { el, t, slider } from './common.js';
import { TimePlot } from './plot.js';
import { css, PALETTE } from './draw.js';
import { gauss, rng } from './corridor.js';

export function simulate(p, T = 8, dt = 0.025) {
  const v = p.v ?? 0.15, track = 0.125, W = 0.25;
  let y = p.y0 ?? 0.04, th = p.th0 ?? 0, integ = 0, prev = null, dfil = 0;
  const r = rng(7), delaySteps = Math.round((p.delay ?? 0) / dt), hist = [];
  const out = { t: [], y: [], e: [], turn: [], crash: false };
  for (let k = 0; k * dt <= T; k++) {
    const cl = Math.cos(Math.PI / 4 - th), cr = Math.cos(Math.PI / 4 + th);
    const dl = cl > 0.1 ? Math.min(1.3, (W / 2 - y) / cl) : 1.3, dr = cr > 0.1 ? Math.min(1.3, (W / 2 + y) / cr) : 1.3;
    let e = dl - dr + (p.noise ?? 0) * gauss(r) + (p.bias ?? 0);
    hist.push(e); e = hist[Math.max(0, hist.length - 1 - delaySteps)];
    integ = Math.max(-0.5, Math.min(0.5, integ + e * dt));
    const raw = prev === null ? 0 : (e - prev) / dt; prev = e;
    dfil = 0.3 * raw + 0.7 * dfil;
    let turn = (p.kp ?? 0) * e + (p.ki ?? 0) * integ + (p.kd ?? 0) * dfil;
    turn = Math.max(-v, Math.min(v, turn));
    const om = 2 * turn / track + (p.mismatch ?? 0) * v / track;
    th += om * dt; y += v * Math.sin(th) * dt;
    out.t.push(k * dt); out.y.push(y * 1000); out.e.push(e * 1000); out.turn.push(turn);
    if (Math.abs(y) > W / 2 - 0.035) { out.crash = true; break; }
  }
  return out;
}

/** opts: {sliders: ['kp','ki','kd','delay','noise','mismatch'], init: {...}, compare: boolean} */
export function lateralDemo(box, opts) {
  const p = { kp: 1, ki: 0, kd: 0, delay: 0, noise: 0, mismatch: 0, y0: 0.04, ...opts.init };
  const ctrl = el('div', { class: 'grid-3' });
  const plot = new TimePlot(box, { title: t('tut.demo_plot'), series: [{ name: t('tut.demo_now'), color: PALETTE[0] }, { name: t('tut.demo_ref'), color: '#adb5bd', dash: [4, 3] }], keepAll: true, height: 170 });
  const msg = el('p', { class: 'muted' });
  box.prepend(ctrl); box.append(msg);
  const ref = simulate(p);
  const DEF = { kp: [0, 10, 0.05, 'Kp', ''], ki: [0, 5, 0.05, 'Ki', ''], kd: [0, 2, 0.01, 'Kd', ''], delay: [0, 0.3, 0.01, t('tut.delay'), 's'], noise: [0, 0.02, 0.001, t('tut.noise'), 'm'], mismatch: [-0.1, 0.1, 0.005, t('tut.mismatch'), ''] };
  const run = () => {
    const s = simulate(p);
    plot.setData(0, s.t, s.y); plot.setData(1, ref.t, ref.y); plot.draw();
    msg.textContent = s.crash ? t('tut.demo_crash') : t('tut.demo_final', { y: s.y[s.y.length - 1].toFixed(1) });
  };
  for (const k of opts.sliders) {
    const [mn, mx, st, label, unit] = DEF[k];
    ctrl.append(slider({ label, unit, min: mn, max: mx, step: st, value: p[k], onInput: (v) => { p[k] = v; run(); } }));
  }
  run();
}

/** the geometry demo: drag the robot, see both distances and the error */
export function geometryDemo(box) {
  const c = el('canvas'); const wrap = el('div', { class: 'canvas-wrap' }, c); box.append(wrap);
  const info = el('p', { class: 'mono' }); box.append(info);
  let y = 0.03, th = 0.2;
  const ys = slider({ label: t('tut.pos'), unit: 'mm', min: -60, max: 60, step: 1, value: 30, onInput: (v) => { y = v / 1000; draw(); } });
  const ts = slider({ label: t('tut.angle'), unit: '°', min: -30, max: 30, step: 1, value: 11, onInput: (v) => { th = v * Math.PI / 180; draw(); } });
  box.prepend(el('div', { class: 'grid-2' }, ys, ts));
  function draw() {
    const w = wrap.clientWidth || 500, h = 220, d = devicePixelRatio || 1; c.width = w * d; c.height = h * d; c.style.height = h + 'px';
    const g = c.getContext('2d'); g.setTransform(d, 0, 0, d, 0, 0); g.clearRect(0, 0, w, h);
    const S = 900, X = (x) => w / 2 + x * S, Y = (v) => h / 2 - v * S, W = 0.1;
    g.strokeStyle = css('--wall'); g.lineWidth = 3; g.beginPath(); g.moveTo(0, Y(W)); g.lineTo(w, Y(W)); g.moveTo(0, Y(-W)); g.lineTo(w, Y(-W)); g.stroke();
    const dl = (W - y) / Math.cos(Math.PI / 4 - th), dr = (W + y) / Math.cos(Math.PI / 4 + th);
    for (const [a, dd, col] of [[th + Math.PI / 4, dl, PALETTE[2]], [th - Math.PI / 4, dr, PALETTE[1]]]) {
      g.strokeStyle = col; g.lineWidth = 2; g.beginPath(); g.moveTo(X(0), Y(y)); g.lineTo(X(Math.cos(a) * dd), Y(y + Math.sin(a) * dd)); g.stroke();
    }
    g.save(); g.translate(X(0), Y(y)); g.rotate(-th); g.fillStyle = css('--robot') + '55'; g.strokeStyle = css('--robot'); g.fillRect(-25, -22, 60, 44); g.strokeRect(-25, -22, 60, 44); g.restore();
    info.textContent = `${t('tut.left')} ${(dl * 1000).toFixed(0)} mm   ${t('tut.right')} ${(dr * 1000).toFixed(0)} mm   ${t('tut.error')} = ${((dl - dr) * 1000).toFixed(0)} mm`;
  }
  new ResizeObserver(draw).observe(wrap);
  draw();
}
