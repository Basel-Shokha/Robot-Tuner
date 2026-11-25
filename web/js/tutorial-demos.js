// tutorial-demos.js - small interactive demos for the PID tutorial. The model is in tutorial-sim.js (pure, tested);
// the full simulator is in the playground.
import { el, t, slider } from './common.js';
import { TimePlot } from './plot.js';
import { css, PALETTE } from './draw.js';
import { simulate, corridorCurve, settleDistance, GRAPH_PRESETS, wheelCommands } from './tutorial-sim.js';
export { simulate };

/** opts: {sliders: ['kp','ki','kd','delay','noise','mismatch','alpha'], init: {...}, model: {...extra simulate params}} */
export function lateralDemo(box, opts) {
  const p = { kp: 1, ki: 0, kd: 0, delay: 0, noise: 0, mismatch: 0, y0: 0.04, ...opts.model, ...opts.init };
  const ctrl = el('div', { class: 'grid-3' });
  const showU = opts.show === 'turn';
  const plot = new TimePlot(box, { title: t(showU ? 'tut.demo_plot_u' : 'tut.demo_plot'), series: [{ name: t('tut.demo_now'), color: PALETTE[0] }, { name: t('tut.demo_ref'), color: '#9aa6b0', dash: [4, 3] }], keepAll: true, height: 170 });
  const msg = el('p', { class: 'muted' });
  box.prepend(ctrl); box.append(msg);
  const ref = simulate(p);
  const DEF = { kp: [0, 10, 0.05, 'Kp', ''], ki: [0, 5, 0.05, 'Ki', ''], kd: [0, 2, 0.01, 'Kd', ''], delay: [0, 0.3, 0.01, t('tut.delay'), 's'],
    noise: [0, 0.02, 0.001, t('tut.noise'), 'm'], mismatch: [-0.1, 0.1, 0.005, t('tut.mismatch'), ''], alpha: [0.05, 1, 0.05, t('tut.alpha'), ''] };
  const run = () => {
    const s = simulate(p), k = showU ? 'turn' : 'y', sc = showU ? 1000 : 1;
    plot.setData(0, s.t, s[k].map((v) => v * sc)); plot.setData(1, ref.t, ref[k].map((v) => v * sc)); plot.draw();
    msg.textContent = s.crash ? t('tut.demo_crash') : t('tut.demo_final', { y: s.y[s.y.length - 1].toFixed(1) });
  };
  for (const k of opts.sliders) {
    const [mn, mx, st, label, unit] = DEF[k];
    ctrl.append(slider({ label, unit, min: mn, max: mx, step: st, value: p[k], onInput: (v) => { p[k] = v; run(); } }));
  }
  run();
}

/** "distance from the centre vs distance driven": sliders for Kp, Ki, Kd plus the four example settings */
export function corridorGraph(box) {
  const g = { ...GRAPH_PRESETS.good };
  const plot = new TimePlot(box, { title: t('tut.g_title'), tUnit: ' cm', keepAll: true, fixed: true, min: -8, max: 8, height: 230,
    series: [{ name: t('tut.g_yours'), color: PALETTE[0], width: 2.2 }, { name: t('tut.g_good'), color: '#9aa6b0', dash: [4, 3] }] });
  const msg = el('p', { class: 'muted', 'aria-live': 'polite' });
  const sl = {
    kp: slider({ label: 'Kp', min: 0, max: 5, step: 0.05, value: g.kp, onInput: (v) => { g.kp = v; run(); } }),
    ki: slider({ label: 'Ki', min: 0, max: 1, step: 0.01, value: g.ki, onInput: (v) => { g.ki = v; run(); } }),
    kd: slider({ label: 'Kd', min: 0, max: 3, step: 0.05, value: g.kd, onInput: (v) => { g.kd = v; run(); } }),
  };
  const presets = el('div', { class: 'row' }, el('span', { class: 'muted', text: t('tut.g_try') }));
  for (const k of Object.keys(GRAPH_PRESETS)) {
    presets.append(el('button', { class: 'small', type: 'button', text: t('tut.g_p_' + k), onclick: () => {
      Object.assign(g, GRAPH_PRESETS[k]); for (const q of ['kp', 'ki', 'kd']) sl[q].set(g[q], false); run();
    } }));
  }
  box.prepend(el('div', { class: 'grid-3' }, sl.kp, sl.ki, sl.kd), presets);
  box.append(el('small', { class: 'muted', text: t('tut.g_axis') }), msg);
  const ref = corridorCurve(GRAPH_PRESETS.good);
  function run() {
    const c = corridorCurve(g);
    plot.setData(0, c.x, c.y); plot.setData(1, ref.x, ref.y); plot.draw();
    const d = settleDistance(c);
    const late = Math.max(...c.y.slice(-Math.floor(c.y.length / 4)).map(Math.abs));
    msg.textContent = Number.isFinite(d) ? t('tut.g_settled', { d: d.toFixed(0) })
      : late > 3 ? t('tut.g_wobble') : t('tut.g_slow', { y: late.toFixed(1) });
  }
  run();
}

/** the wheel mixer: e -> u -> base -/+ u, with the speed range and clipping */
export function wheelDemo(box) {
  const p = { u: 0.03, frac: 0.75, vmin: 0.05, vmax: 0.3 };
  const out = el('div', { class: 'wheels-demo' });
  const bar = (label) => {
    const fill = el('div', { class: 'wd-fill' }), val = el('span', { class: 'mono' });
    const row = el('div', { class: 'wd-row' }, el('span', { class: 'wd-lab', text: label }), el('div', { class: 'wd-bar' }, fill, el('i', { class: 'wd-min' }), el('i', { class: 'wd-max' })), val);
    out.append(row); return { fill, val, row };
  };
  const L = bar(t('tut.w_left')), B = bar('base'), Rr = bar(t('tut.w_right'));
  const note = el('p', { class: 'muted', 'aria-live': 'polite' });
  box.append(el('div', { class: 'grid-2' },
    slider({ label: t('tut.w_u'), unit: 'm/s', min: -0.3, max: 0.3, step: 0.005, value: p.u, onInput: (v) => { p.u = v; draw(); } }),
    slider({ label: 'speed_frac', min: 0, max: 1, step: 0.05, value: p.frac, onInput: (v) => { p.frac = v; draw(); } })), out, note);
  function draw() {
    const w = wheelCommands(p.u, p);
    const pos = (v) => Math.max(0, Math.min(100, v / 0.35 * 100));
    for (const [b, raw, clip] of [[L, w.left, w.leftClip], [B, w.base, w.base], [Rr, w.right, w.rightClip]]) {
      b.fill.style.width = pos(clip) + '%';
      b.fill.classList.toggle('clipped', raw !== clip);
      b.val.textContent = raw !== clip ? `${raw.toFixed(3)} → ${clip.toFixed(3)} m/s` : `${clip.toFixed(3)} m/s`;
    }
    out.style.setProperty('--vmin', pos(p.vmin) + '%'); out.style.setProperty('--vmax', pos(p.vmax) + '%');
    note.textContent = w.clipped ? t('tut.w_clipped') : t('tut.w_ok', { d: (2 * p.u).toFixed(3) });
  }
  draw();
}

/** the geometry demo: move the robot, see both distances and the error */
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
    const e = (dl - dr) * 1000;
    info.textContent = `${t('tut.left')} ${(dl * 1000).toFixed(0)} mm   ${t('tut.right')} ${(dr * 1000).toFixed(0)} mm   e = ${e.toFixed(0)} mm  → ${t(Math.abs(e) < 3 ? 'tut.e_zero' : e > 0 ? 'tut.e_pos' : 'tut.e_neg')}`;
  }
  new ResizeObserver(draw).observe(wrap);
  draw();
}
