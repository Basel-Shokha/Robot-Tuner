// corridor-page.js - 3. CORRIDOR SIMULATOR: environment, wall errors, track editor, start pose, noise.
// The settings are saved (rt.env) and used by the playground and the trainer.
import { page, $, $$, el, t, slider, toggle, pickFile, toast } from './common.js';
import { loadRobot } from './store.js';
import { loadEngine, S } from './engine.js';
import { loadEnv, saveEnv, TRACK_PRESETS, parseTrackFile } from './corridor.js';
import { Scene, Runner, drawScene } from './simview.js';
import { View, css } from './draw.js';
import { TimePlot } from './plot.js';
import { fmt } from './i18n.js';

await page();
const engine = await loadEngine();
const robot = loadRobot();
let env = loadEnv();
const view = new View($('#cv'), { aspect: 0.45, minH: 300, maxH: 560 });
let scene, runner, running = false, drawing = false;
const plots = {
  ce: new TimePlot($('#plots'), { title: t('plot.center_err'), series: [{ name: t('plot.center_err_s'), color: '#1f6feb' }], window: 20 }),
  he: new TimePlot($('#plots'), { title: t('plot.heading_err'), series: [{ name: t('plot.heading_err_s'), color: '#e8590c' }], window: 20 }),
};

function changed(rebuild = true) { saveEnv(env); if (rebuild) restart(); }

function restart() {
  running = false; $('#run').textContent = t('sim.run');
  if (runner) runner.destroy();
  scene = new Scene(env);
  runner = new Runner(engine, robot, env, { color: css('--robot') });
  scene.update([runner], true);
  const pose = scene.start(robot, env.seed);
  runner.reset(pose, env.seed);
  scene.update([runner], true);
  view.userMoved = false;
  plots.ce.clear(); plots.he.clear();
  startNote(pose);
  render();
}
function startNote(pose) {
  const n = pose.note === 'angle' ? t('corr.start_adjusted_angle', { a: fmt(pose.angle, 0) }) : pose.note === 'offset' ? t('corr.start_adjusted_offset') : pose.note === 'impossible' ? t('corr.start_impossible') : '';
  $('#start-note').textContent = n;
}
function renderBase() {
  drawScene(view, scene, [runner], { crashLabel: t('sim.crash') });
  const st = runner.stats(0.0);
  $('#ov').textContent = `${t('sim.time')} ${fmt(st.t, 1)} s · ${t('sim.dist')} ${fmt(st.dist, 2)} m · ${t('sim.mean_ce')} ${fmt(st.meanCe, 1)} mm` + (st.crashed ? ' · ' + t('sim.crashed') : '') + (st.finished ? ' · ' + t('sim.finished') : '');
  plots.ce.setData(0, runner.h.t, runner.h.ce); plots.he.setData(0, runner.h.t, runner.h.he);
  plots.ce.draw(); plots.he.draw();
}
let lastT = 0, acc = 0;
function loop(ts) {
  if (running) {
    acc += Math.min(0.1, (ts - lastT) / 1000) * +$('#rate').value;
    const n = Math.floor(acc / runner.dt); acc -= n * runner.dt;
    runner.step(n);
    scene.update([runner]);
    if (!runner.alive || runner.last[S.t] > env.duration_s) { running = false; $('#run').textContent = t('sim.run'); }
    render();
  }
  lastT = ts;
  requestAnimationFrame(loop);
}
view.onChange = () => render();
$('#run').addEventListener('click', () => {
  if (!runner.alive) restart();
  running = !running; $('#run').textContent = running ? t('sim.pause') : t('sim.run');
});
$('#reset').addEventListener('click', restart);

// ---- settings panels ---------------------------------------------------------------------------------
function buildSide() {
  $$('[data-mode]').forEach((b) => b.setAttribute('aria-selected', b.dataset.mode === env.mode));
  $('#env-endless').hidden = env.mode !== 'endless';
  $('#env-track').hidden = env.mode !== 'track';
  // endless
  const E = $('#env-endless'); E.innerHTML = '';
  E.append(el('h3', { text: t('corr.endless_h') }),
    slider({ key: 'corr.width', unit: 'mm', min: 120, max: 600, step: 5, value: env.width_mm, onInput: (v) => { env.width_mm = v; changed(); } }),
    el('div', { class: 'row' }, el('span', { text: t('corr.seed') + ': ' + env.seed }), el('button', { class: 'small', text: t('corr.new_corridor'), onclick: () => { env.seed = 1 + Math.floor(Math.random() * 99999); changed(); buildSide(); } })));
  // track
  const T = $('#env-track'); T.innerHTML = '';
  const sel = el('select', {}, ...Object.keys(TRACK_PRESETS).map((k) => el('option', { value: k, text: t('track.' + k) })), el('option', { value: 'custom', text: t('track.custom') }));
  sel.value = env.track.imported ? 'custom' : env.track.points ? 'custom' : env.track.preset;
  sel.addEventListener('change', () => { env.track.imported = null; if (sel.value !== 'custom') { env.track.preset = sel.value; env.track.points = null; } else env.track.points = (TRACK_PRESETS[env.track.preset] || TRACK_PRESETS.straight).map((p) => p.slice()); changed(); });
  T.append(el('h3', { text: t('corr.track_h') }), el('label', { class: 'field' }, el('span', { text: t('corr.track_shape') }), sel),
    slider({ key: 'corr.width', unit: 'mm', min: 120, max: 600, step: 5, value: env.track.width_mm, onInput: (v) => { env.track.width_mm = v; changed(); } }),
    slider({ key: 'corr.radius', unit: 'mm', min: 50, max: 1500, step: 10, value: env.track.radius_mm, onInput: (v) => { env.track.radius_mm = v; changed(); } }),
    el('div', { class: 'row' },
      el('button', { class: 'small', text: drawing ? t('corr.draw_done') : t('corr.draw'), onclick: () => { drawing = !drawing; if (drawing && !env.track.points) env.track.points = [[0, 0], [1, 0]]; $('#draw-help').hidden = !drawing; changed(); buildSide(); } }),
      el('button', { class: 'small', text: t('corr.import_track'), onclick: async () => {
        const f = await pickFile('.track,.txt'); if (!f) return;
        try { env.track.imported = parseTrackFile(f.text); changed(); buildSide(); toast(t('builder.imported', { name: f.name }), 'ok'); } catch (e) { toast(e.message, 'bad'); }
      } })),
    el('small', { class: 'muted', text: t('corr.track_note') }));
  // errors
  const R = $('#env-errors'); R.innerHTML = '';
  R.append(el('h3', { text: t('corr.errors_h') }), el('small', { class: 'muted', text: t('corr.errors_note') }));
  const er = env.errors;
  const errRow = (k, fields) => {
    const box = el('fieldset', {}, el('legend', {}, toggle({ key: 'err.' + k, value: er[k].on, onChange: (v) => { er[k].on = v; changed(); } })));
    for (const f of fields) box.append(slider({ key: `err.${k}.${f[0]}`, unit: f[1], min: f[2], max: f[3], step: f[4], value: er[k][f[0]], onInput: (v) => { er[k][f[0]] = v; changed(); } }));
    box.append(el('small', { class: 'muted', text: t(`err.${k}.hint`) }));
    return box;
  };
  R.append(errRow('jitter', [['mm', 'mm', 0, 10, 0.5]]), errRow('tilt', [['deg', '°', 0, 5, 0.1]]));
  if (env.mode === 'endless') R.append(errRow('bumps', [['mm', 'mm', 2, 40, 1], ['per_m', '1/m', 0.05, 2, 0.05]]), errRow('gaps', [['mm', 'mm', 20, 200, 5], ['per_m', '1/m', 0.05, 2, 0.05]]), errRow('steps', [['mm', 'mm', 2, 60, 1], ['per_m', '1/m', 0.05, 2, 0.05]]));
  // start pose
  const P = $('#env-start'); P.innerHTML = '';
  P.append(el('h3', { text: t('corr.start_h') }),
    slider({ key: 'corr.start_angle', unit: '°', min: -80, max: 80, step: 1, value: env.start.angle_deg, onInput: (v) => { env.start.angle_deg = v; changed(); } }),
    slider({ key: 'corr.start_offset', unit: 'mm', min: -150, max: 150, step: 1, value: env.start.offset_mm, onInput: (v) => { env.start.offset_mm = v; changed(); } }),
    toggle({ key: 'corr.start_random', value: env.start.random, onChange: (v) => { env.start.random = v; changed(); } }),
    el('div', { class: 'row' }, el('button', { class: 'small', text: t('corr.new_random_start'), onclick: () => { env.seed = 1 + Math.floor(Math.random() * 99999); changed(); } })),
    el('small', { class: 'muted', id: 'start-note' }));
  // noise
  const N = $('#env-noise'); N.innerHTML = '';
  const nz = env.noise;
  N.append(el('h3', { text: t('corr.noise_h') }),
    slider({ key: 'noise.global', unit: '×', min: 0, max: 3, step: 0.1, value: nz.global, onInput: (v) => { nz.global = v; changed(); }, hint: t('noise.global_hint') }),
    slider({ key: 'noise.sensor', unit: '×', min: 0, max: 5, step: 0.1, value: nz.sensor, onInput: (v) => { nz.sensor = v; changed(); } }),
    slider({ key: 'noise.encoder', unit: '×', min: 0, max: 5, step: 0.1, value: nz.encoder, onInput: (v) => { nz.encoder = v; changed(); } }),
    slider({ key: 'noise.mismatch', unit: '%', min: -15, max: 15, step: 0.5, value: nz.mismatch_pct, onInput: (v) => { nz.mismatch_pct = v; changed(); }, hint: t('noise.mismatch_hint') }),
    slider({ key: 'noise.delay', unit: 'ms', min: 0, max: 300, step: 5, value: nz.delay_ms, onInput: (v) => { nz.delay_ms = v; changed(); }, hint: t('noise.delay_hint') }),
    toggle({ key: 'noise.angle_bias', value: nz.angle_bias, onChange: (v) => { nz.angle_bias = v; changed(); } }),
    slider({ key: 'corr.duration', unit: 's', min: 5, max: 240, step: 5, value: env.duration_s, onInput: (v) => { env.duration_s = v; changed(false); } }));
}
$$('[data-mode]').forEach((b) => b.addEventListener('click', () => { env.mode = b.dataset.mode; drawing = false; $('#draw-help').hidden = true; changed(); buildSide(); }));

// track drawing: click adds a point, drag moves the nearest point, right click / long press removes it
let dragPt = -1;
view.onPointerDown = (e) => {
  if (!drawing || env.mode !== 'track') return false;
  const [x, y] = view.eventWorld(e), pts = env.track.points;
  let best = -1, bd = 12 / view.scale;
  pts.forEach((p, i) => { const d = Math.hypot(p[0] - x, p[1] - y); if (d < bd) { bd = d; best = i; } });
  if (e.button === 2 || e.ctrlKey) { if (best >= 0 && pts.length > 2) { pts.splice(best, 1); changed(); } return true; }
  if (best < 0) { pts.push([+x.toFixed(3), +y.toFixed(3)]); best = pts.length - 1; changed(); }
  dragPt = best; view.c.setPointerCapture(e.pointerId); return true;
};
view.onPointerMove = (e) => {
  if (dragPt < 0) return false;
  const [x, y] = view.eventWorld(e); env.track.points[dragPt] = [+x.toFixed(3), +y.toFixed(3)];
  scene = new Scene(env); render(); drawPoints(); return true;
};
view.onPointerUp = () => { if (dragPt >= 0) { dragPt = -1; changed(); } };
$('#cv').addEventListener('contextmenu', (e) => { if (drawing) e.preventDefault(); });
function drawPoints() {
  if (!drawing || !env.track.points) return;
  view.polyline(env.track.points, css('--accent'), 1, [3, 3]);
  env.track.points.forEach((p) => view.dot(p[0], p[1], 5, css('--accent')));
}
function render() { renderBase(); drawPoints(); }

buildSide();
restart();
requestAnimationFrame(loop);
