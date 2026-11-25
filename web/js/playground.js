// playground.js - 5. PLAYGROUND: live gains while the robot drives, presets, sweep with ghost robots.
import { page, $, el, t, slider, toast } from './common.js';
import { loadRobot, saveRobot, cruiseSpeed } from './store.js';
import { loadEngine, S } from './engine.js';
import { loadEnv } from './corridor.js';
import { Scene, Runner, drawScene } from './simview.js';
import { View, css, PALETTE } from './draw.js';
import { TimePlot } from './plot.js';
import { PRESETS, PRESET_IDS } from './presets.js';
import { fmt } from './i18n.js';

await page();
const engine = await loadEngine();
let stored = loadRobot();
let robot = JSON.parse(JSON.stringify(stored));
let env = loadEnv();
const view = new View($('#cv'), { aspect: 0.42, minH: 280, maxH: 520 });
let scene, runners = [], running = false, sweep = null, seed = 1;
const P = $('#plots');
const plots = {
  ce: new TimePlot(P, { title: t('plot.center_err'), series: [], window: 15 }),
  he: new TimePlot(P, { title: t('plot.heading_err'), series: [], window: 15 }),
  wheels: new TimePlot(P, { title: t('pg.plot_wheels'), series: [], window: 15 }),
  turn: new TimePlot(P, { title: t('pg.plot_turn'), series: [], window: 15 }),
  pid: new TimePlot(P, { title: t('pg.plot_pid'), series: [], window: 15 }),
};

const params = new URLSearchParams(location.search);
const sliders = {};
function buildGains() {
  const G = $('#gains'); G.innerHTML = '';
  const mx = robot.motor.max_speed_mps, k0 = Math.max(0.5, stored.pid.kp);
  G.append(el('h3', { text: t('pg.gains_h') }),
    sliders.kp = slider({ key: 'pg.kp', min: 0, max: Math.max(100, k0 * 30), step: 'any', log: true, value: +robot.pid.kp.toPrecision(3), onInput: (v) => live('kp', v), hint: t('pg.kp_hint') }),
    sliders.ki = slider({ key: 'pg.ki', min: 0, max: 20, step: 'any', log: true, value: +robot.pid.ki.toPrecision(3), onInput: (v) => live('ki', v), hint: t('pg.ki_hint') }),
    sliders.kd = slider({ key: 'pg.kd', min: 0, max: Math.max(20, stored.pid.kd * 30), step: 'any', log: true, value: +robot.pid.kd.toPrecision(3), onInput: (v) => live('kd', v), hint: t('pg.kd_hint') }),
    sliders.speed = slider({ key: 'pg.speed_frac', min: 0, max: 1, step: 0.01, value: +robot.pid.speed_frac.toFixed(2), onInput: (v) => live('speed_frac', v), hint: t('pg.speed_frac_hint') }),
    el('p', { class: 'muted', id: 'cruise-line' }),
    sliders.offset = slider({ key: 'pg.offset', unit: 'mm', min: -200, max: 200, step: 5, value: robot.pid.offset_mm || 0, onInput: (v) => { live('offset_mm', v); }, hint: t('pg.offset_hint') }),
    el('small', { class: 'muted', text: t('pg.units') }));
  cruiseLine();
}
function live(k, v) {
  robot.pid[k] = v;
  cruiseLine();
  if (!sweep) for (const r of runners) r.configure(robot);
}
/** base = v_min + speed_frac (v_max - v_min): what the slider means in m/s for this robot */
function cruiseLine() {
  const e = document.getElementById('cruise-line'); if (!e) return;
  e.textContent = t('pg.cruise_line', { v: fmt(cruiseSpeed(robot), 3), vmin: fmt(robot.motor.v_min_mps || 0, 3), vmax: fmt(robot.motor.max_speed_mps, 3), room: fmt(robot.motor.max_speed_mps - cruiseSpeed(robot), 3) });
}

function legend() {
  $('#legend').innerHTML = runners.length > 1 ? runners.map((r) => `<span><i style="background:${r.color}"></i>${r.label}</span>`).join('') : '';
}
function restart() {
  running = false; $('#run').textContent = t('sim.run');
  for (const r of runners) r.destroy();
  scene = new Scene(env);
  const variants = sweep ? sweep.values.map((v) => ({ r: patch(robot, sweep.param, v), label: `${t('pg.p_' + sweep.param)} = ${v}` })) : [{ r: robot, label: '' }];
  runners = variants.map((x, i) => new Runner(engine, x.r, env, { color: PALETTE[i % PALETTE.length], label: x.label }));
  scene.update(runners, true);
  const pose = scene.start(robot, seed);
  for (const r of runners) r.reset(pose, seed, scene.loopLength);  // same start AND same noise sequence for every ghost
  view.userMoved = false;
  const set = (p, keys, names) => p.setSeries(runners.length > 1 ? runners.map((r) => ({ name: r.label, color: r.color })) : keys.map((k, i) => ({ name: names[i], color: PALETTE[i] })));
  set(plots.ce, ['ce'], [t('plot.center_err_s')]);
  if (runners.length === 1) plots.ce.setSeries([{ name: t('plot.center_err_s'), color: PALETTE[0] }, { name: t('pg.target'), color: '#9aa6b0', dash: [5, 4] }]); set(plots.he, ['he'], [t('plot.heading_err_s')]);
  plots.wheels.setSeries([{ name: t('pg.cmd_l'), color: PALETTE[0] }, { name: t('pg.cmd_r'), color: PALETTE[1] }, { name: t('pg.real_l'), color: PALETTE[0], dash: [3, 3] }, { name: t('pg.real_r'), color: PALETTE[1], dash: [3, 3] }]);
  plots.pid.setSeries([{ name: 'P', color: PALETTE[0] }, { name: 'I', color: PALETTE[2] }, { name: 'D', color: PALETTE[3] }]);
  plots.turn.setSeries(runners.length > 1 ? runners.flatMap((r) => [{ name: `${r.label}: ${t('pg.turn_asked')}`, color: r.color }, { name: `${r.label}: ${t('pg.turn_real')}`, color: r.color, dash: [3, 3] }])
    : [{ name: t('pg.turn_asked'), color: PALETTE[0] }, { name: t('pg.turn_real'), color: PALETTE[1], dash: [3, 3] }]);
  legend(); render(); table();
  $('#env-sum').textContent = t('pg.env_sum', { mode: t(env.mode === 'track' ? 'corr.mode_track' : 'corr.mode_endless'), w: env.width_mm, a: env.start.angle_deg, o: env.start.offset_mm, n: env.noise.global });
}
function patch(r, k, v) { const c = JSON.parse(JSON.stringify(r)); if (k === 'speed') c.pid.speed_frac = v; else if (k === 'offset') c.pid.offset_mm = v; else c.pid[k] = v; return c; }

function render() {
  drawScene(view, scene, runners, { crashLabel: t('sim.crash'), clip: true });
  const L = runners[0], st = L.stats(1.0), s = L.last;
  // clipping lesson: which wheel is cut at v_max, and the turn the PID asked for vs the turn the wheels can give
  const clipTxt = s && (s[S.clip_l] || s[S.clip_r])
    ? ' · ' + t('pg.clipped', { w: t(s[S.clip_l] && s[S.clip_r] ? 'pg.both_wheels' : s[S.clip_l] ? 'pg.left_wheel' : 'pg.right_wheel'), a: fmt(s[S.turn], 3), r: fmt(s[S.real_turn], 3) }) : '';
  $('#ov').textContent = `${t('sim.time')} ${fmt(st.t, 1)} s · ${t('sim.mean_ce')} ${fmt(st.meanCe, 1)} mm` + (scene.loopLength ? ' · ' + t('sim.laps', { n: st.laps }) : '') + clipTxt + (st.crashed ? ' · ' + t('sim.crashed') : '');
  $('#ov').classList.toggle('clipping', !!clipTxt);
  if (runners.length > 1) runners.forEach((r, i) => { plots.ce.setData(i, r.h.t, r.h.ce); plots.he.setData(i, r.h.t, r.h.he); });
  else {
    plots.ce.setData(0, L.h.t, L.h.ce); plots.he.setData(0, L.h.t, L.h.he);
    const o = robot.pid.offset_mm || 0; plots.ce.setData(1, L.h.t.length ? [L.h.t[0], L.h.t[L.h.t.length - 1]] : [], L.h.t.length ? [o, o] : []);
  }
  const h = L.h;
  plots.wheels.setData(0, h.t, h.vl); plots.wheels.setData(1, h.t, h.vr); plots.wheels.setData(2, h.t, h.wl); plots.wheels.setData(3, h.t, h.wr);
  plots.pid.setData(0, h.t, h.p); plots.pid.setData(1, h.t, h.i); plots.pid.setData(2, h.t, h.d);
  if (runners.length > 1) runners.forEach((r, i) => { plots.turn.setData(2 * i, r.h.t, r.h.ta); plots.turn.setData(2 * i + 1, r.h.t, r.h.tr); });
  else { plots.turn.setData(0, h.t, h.ta); plots.turn.setData(1, h.t, h.tr); }
  const pct = (r) => { const q = r.h, n = q.t.length; return n ? 100 * q.cl.reduce((a, x, k) => a + (x || q.cr[k] ? 1 : 0), 0) / n : NaN; };
  $('#clip-stat').textContent = !h.t.length ? '' : runners.length > 1
    ? runners.map((r) => `${r.label}: ${t('pg.clip_stat', { p: fmt(pct(r), 0) })}`).join(' · ') : t('pg.clip_stat', { p: fmt(pct(L), 0) });
  Object.values(plots).forEach((p) => p.draw());
}
function table() {
  const T = $('#table'); T.hidden = runners.length < 2; if (T.hidden) return;
  T.innerHTML = `<h3>${t('pg.sweep_results')}</h3><table><tr><th></th><th class="num">${t('pg.col_mean')}</th><th class="num">${t('pg.col_max')}</th><th class="num">${t('pg.col_head')}</th><th>${t('pg.col_result')}</th></tr>` +
    runners.map((r) => { const s = r.stats(1.0); return `<tr><td><i style="display:inline-block;width:10px;height:10px;background:${r.color}"></i> ${r.label}</td><td class="num">${fmt(s.meanCe, 1)}</td><td class="num">${fmt(s.maxCe, 0)}</td><td class="num">${fmt(s.meanHe, 1)}</td><td>${s.crashed ? '<span class="pill bad">' + t('sim.crashed') + '</span>' : '<span class="pill ok">OK</span>'}</td></tr>`; }).join('') + '</table>';
}
let lastT = 0, acc = 0, tick = 0;
function loop(ts) {
  if (running) {
    acc += Math.min(0.1, (ts - lastT) / 1000) * +$('#rate').value;
    const dt = runners[0].dt, n = Math.floor(acc / dt); acc -= n * dt;
    for (const r of runners) r.step(n);
    scene.update(runners);
    if (runners.every((r) => !r.alive) || runners[0].last[S.t] > env.duration_s) { running = false; $('#run').textContent = t('sim.run'); }
    render(); if (++tick % 15 === 0 || !running) table();
  }
  lastT = ts; requestAnimationFrame(loop);
}
view.onChange = render;
$('#run').addEventListener('click', () => { if (runners.every((r) => !r.alive)) restart(); running = !running; $('#run').textContent = running ? t('sim.pause') : t('sim.run'); });
$('#reset').addEventListener('click', () => { seed++; restart(); });
$('#save-gains').addEventListener('click', () => { stored.pid = { ...stored.pid, kp: robot.pid.kp, ki: robot.pid.ki, kd: robot.pid.kd, speed_frac: robot.pid.speed_frac, offset_mm: robot.pid.offset_mm || 0 }; saveRobot(stored); toast(t('pg.saved'), 'ok'); });

// presets
for (const id of PRESET_IDS) $('#presets').append(el('button', { class: 'small', text: t('preset.' + id), onclick: () => applyPreset(id) }));
function applyPreset(id) {
  let sw;
  [robot, env, sw] = PRESETS[id](stored, loadEnv());
  sweep = sw || null; buildGains(); buildSweep();
  $('#preset-desc').textContent = t('preset.' + id + '.desc');
  restart(); running = true; $('#run').textContent = t('sim.pause');
}

// sweep
function buildSweep() {
  const W = $('#sweep'); W.innerHTML = '';
  const sel = el('select', {}, ...['kp', 'kd', 'ki', 'speed', 'offset'].map((k) => el('option', { value: k, text: t('pg.p_' + k) })));
  const vals = el('input', { style: 'flex:1', value: '' });
  const fill = () => {
    if (sel.value === 'offset') { vals.value = '-60, -30, 0, 30, 60'; return; }
    if (sel.value === 'speed') { vals.value = '0.5, 0.75, 1'; return; }
    const b = robot.pid[sel.value];
    vals.value = [0.25, 0.5, 1, 2, 4].map((m) => +(Math.max(b, sel.value === 'ki' ? 0.01 : 1e-3) * m).toPrecision(2)).join(', ');
  };
  sel.addEventListener('change', fill); fill();
  W.append(el('h3', { text: t('pg.sweep_h') }), el('p', { class: 'muted', text: t('pg.sweep_p') }),
    el('div', { class: 'row' }, sel, vals),
    el('div', { class: 'row' },
      el('button', { class: 'primary small', text: t('pg.sweep_run'), onclick: () => {
        let v = vals.value.split(/[,;\s]+/).map(Number).filter((x) => Number.isFinite(x)).slice(0, 8);
        if (sel.value === 'speed') v = v.map((x) => Math.min(1, Math.max(0, x)));  // speed_frac is 0..1
        if (v.length < 2) return toast(t('pg.sweep_need2'), 'bad');
        sweep = { param: sel.value, values: v }; restart(); running = true; $('#run').textContent = t('sim.pause');
      } }),
      el('button', { class: 'small', text: t('pg.sweep_off'), onclick: () => { sweep = null; restart(); } })));
}

buildGains(); buildSweep();
if (params.get('preset') && PRESETS[params.get('preset')]) applyPreset(params.get('preset')); else restart();
requestAnimationFrame(loop);
