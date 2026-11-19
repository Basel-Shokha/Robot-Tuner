// maze-page.js - 7. MAZE RUNNER SANDBOX
import { page, $, el, t, slider, toggle, toast } from '../common.js';
import { loadRobot, robotToConfig } from '../store.js';
import { loadEngine, S } from '../engine.js';
import { View, css, PALETTE } from '../draw.js';
import { defaultMazeSettings, generateGrid, emptyGrid, construct, constructBlocks, blockIsWall, goalCells, cellCenter, cellOf, minCellMm } from './maze.js';
import { ALGORITHMS, CUSTOM_TEMPLATE } from './algorithms.js';
import * as kit from './mazekit.js';
import { fmt } from '../i18n.js';

await page();
const engine = await loadEngine();
const robot = loadRobot();
let st;
try { st = { ...defaultMazeSettings(), ...JSON.parse(localStorage.getItem('rt.maze') || '{}') }; } catch (e) { st = defaultMazeSettings(); }
const minCell = minCellMm(robot);
if (st.kind === 'thick') st.kind = 'thin';  // old setting name
if (st.kind !== 'blocks' && st.kind !== 'thin') st.kind = 'thin';
if (st.cell_mm < minCell) st.cell_mm = minCell + 40;
if (!(st.block_mm >= minCell)) st.block_mm = minCell + 40;
let algoId = localStorage.getItem('rt.maze.algo') || 'flood_fill';
let customCode = localStorage.getItem('rt.maze.code') || CUSTOM_TEMPLATE;
let noise = 0.5;
let grid = null, built = null, goal = [], run = null, running = false;
const vReal = new View($('#cv-real'), { aspect: 1, minH: 300, maxH: 600 }), vMap = new View($('#cv-map'), { aspect: 1, minH: 300, maxH: 600 });
const saveSt = () => { try { localStorage.setItem('rt.maze', JSON.stringify(st)); } catch (e) { /* ignore */ } };

function newGrid() { grid = generateGrid(st.rows, st.cols, st.seed, st.braid); rebuild(); }
function rebuild() { built = st.kind === 'blocks' ? constructBlocks(grid, st) : construct(grid, st); goal = goalCells(grid, st.goal); saveSt(); reset(); }

function algoFactory() {
  if (algoId !== 'custom') return ALGORITHMS[algoId];
  // eslint-disable-next-line no-new-func
  const create = new Function('kit', customCode + '\nreturn create;')(kit);
  return { create: (ctx) => create(ctx, kit) };
}

/** a run = engine instance + algorithm instance */
function makeRun(g, b, gl) {
  const sim = engine.create();
  sim.config({ ...robotToConfig(robot, { noise }), steer_smooth: 0, emerg_on: 0, tof_glitch_threshold_m: 100 });  // the corridor glitch filter would hide openings
  sim.walls(b.walls); sim.finish(false); sim.external(true);
  const startDir = !g.H[1] || g.H[1][0] ? 1 : 0;  // north if open, else east
  const c0 = cellCenter(b, 0, 0), h0 = startDir === 0 ? Math.PI / 2 : 0;
  sim.reset(c0[0], c0[1], h0, st.seed * 7 + 1, true);
  const ctx = {
    rows: g.rows, cols: g.cols, cell: b.cell, thick: b.thickness, start: { r: 0, c: 0, heading: h0 }, startDir, goal: gl,
    sensors: robot.sensors.map((s) => ({ x: s.x_mm / 1000, y: s.y_mm / 1000, angle: s.angle_deg * Math.PI / 180, min: s.min_mm / 1000, max: s.max_mm / 1000 })),
    track: robot.wheels.track_effective_mm / 1000, vmax: 0.6 * robot.motor.max_speed_mps, dwellTicks: 6, wallKp: 1.0, wallKd: 0.1,
  };
  let algo;
  try { algo = algoFactory().create(ctx); } catch (e) { toast(t('mz.code_err') + ' ' + e.message, 'bad'); algo = { step: () => ({ left: 0, right: 0 }) }; }
  return { sim, algo, ctx, path: [[c0[0], c0[1]]], dist: 0, t: 0, found: false, crashed: false, err: null, last: sim.stateRaw() };
}
const MAX_T = 900;
function stepRun(R, n) {
  for (let k = 0; k < n && !R.found && !R.crashed && R.t < MAX_T && !R.err; k++) {
    const s = R.last;
    const obs = { t: s[S.t], dt: robot.timing.loop_ms / 1000, dists: Array.from(s.subarray(S.dists, S.dists + robot.sensors.length)),
      enc: { l: s[S.enc_l], r: s[S.enc_r] }, encDist: { l: s[S.enc_dist_l], r: s[S.enc_dist_r] } };
    let cmd;
    try { cmd = R.algo.step(obs) || { left: 0, right: 0 }; } catch (e) { R.err = e.message; break; }
    const vm = robot.motor.max_speed_mps;
    R.sim.cmd(Math.max(-vm, Math.min(vm, +cmd.left || 0)), Math.max(-vm, Math.min(vm, +cmd.right || 0)));
    R.sim.step(1);
    R.last = R.sim.stateRaw();
    const x = R.last[S.x], y = R.last[S.y], p = R.path[R.path.length - 1];
    R.dist += Math.hypot(x - p[0], y - p[1]); R.path.push([x, y]); R.t = R.last[S.t];
    if (!R.last[S.alive]) R.crashed = true;
    const c = cellOf(built.cell, x, y);
    if (R.goal.some((gc) => gc[0] === c[0] && gc[1] === c[1])) R.found = true;
  }
  if (R.algo.done && !R.found) R.stopped = true;
}
function mapError(R, g) {
  const m = R.algo.map; if (!m) return null;
  let known = 0, wrong = 0, total = 0;
  for (let y = 1; y < g.rows; y++) for (let x = 0; x < g.cols; x++) { total++; const s = m.state(m.H[y][x]); if (s >= 0) { known++; if (s !== g.H[y][x]) wrong++; } }
  for (let y = 0; y < g.rows; y++) for (let x = 1; x < g.cols; x++) { total++; const s = m.state(m.V[y][x]); if (s >= 0) { known++; if (s !== g.V[y][x]) wrong++; } }
  return { known, wrong, total, coverage: known / total, error: known ? wrong / known : 0 };
}

function reset() {
  running = false; $('#run').textContent = t('sim.run');
  if (run) run.sim.destroy();
  run = makeRun(grid, built, goal); run.goal = goal;
  const W = st.cols * built.cell, H = st.rows * built.cell;
  vReal.fit(-0.05, -0.05, W + 0.05, H + 0.05, 0.02); vMap.fit(-0.05, -0.05, W + 0.05, H + 0.05, 0.02);
  draw(); result();
}
function draw() {
  const c = built.cell;
  // real maze
  vReal.begin();
  for (const g of goal) { const q = cellCenter(built, g[0], g[1]); vReal.poly([[q[0] - c / 2, q[1] - c / 2], [q[0] + c / 2, q[1] - c / 2], [q[0] + c / 2, q[1] + c / 2], [q[0] - c / 2, q[1] + c / 2]], css('--ok') + '30', null); }
  vReal.walls(built.walls, css('--wall'), 1.5);
  vReal.polyline(run.path, css('--path'), 1.5);
  vReal.rays(run.sim.rays());
  vReal.robot(run.sim.footprint(), css('--robot'), 1, 0);
  vReal.scaleBar();
  // belief map
  vMap.begin();
  const m = run.algo.map;
  if (run.algo.visited) for (const k of run.algo.visited) { const [y, x] = k.split(',').map(Number); vMap.poly([[x * c, y * c], [(x + 1) * c, y * c], [(x + 1) * c, (y + 1) * c], [x * c, (y + 1) * c]], css('--accent') + '18', null); }
  for (const g of goal) vMap.poly([[g[1] * c, g[0] * c], [(g[1] + 1) * c, g[0] * c], [(g[1] + 1) * c, (g[0] + 1) * c], [g[1] * c, (g[0] + 1) * c]], css('--ok') + '30', null);
  if (m) {
    const edge = (s, a, b) => { if (s === 1) vMap.polyline([a, b], css('--wall'), 3); else if (s === -1) vMap.polyline([a, b], css('--muted'), 1, [3, 4], 0.5); };
    for (let y = 0; y <= m.rows; y++) for (let x = 0; x < m.cols; x++) edge(m.state(m.H[y][x]), [x * c, y * c], [(x + 1) * c, y * c]);
    for (let y = 0; y < m.rows; y++) for (let x = 0; x <= m.cols; x++) edge(m.state(m.V[y][x]), [x * c, y * c], [x * c, (y + 1) * c]);
  } else vMap.text(c, c, t('mz.no_map'));
  if (run.algo.pose) { const p = run.algo.pose; vMap.dot(p.x, p.y, 5, css('--robot')); vMap.polyline([[p.x, p.y], [p.x + Math.cos(p.th) * c * 0.4, p.y + Math.sin(p.th) * c * 0.4]], css('--robot'), 2); }
  vMap.dot(run.last[S.x], run.last[S.y], 3, css('--path'));
}
function result() {
  const me = mapError(run, grid);
  const status = run.found ? `<span class="pill ok">${t('mz.found')}</span>` : run.crashed ? `<span class="pill bad">${t('sim.crashed')}</span>` : run.err ? `<span class="pill bad">${t('mz.code_err')} ${run.err}</span>` : run.t >= MAX_T ? `<span class="pill warn">${t('mz.timeout')}</span>` : run.stopped ? `<span class="pill warn">${t('mz.gave_up')}</span>` : `<span class="pill">${t('mz.running')}</span>`;
  $('#result').innerHTML = `<div class="row">${status}<span>${t('sim.time')} ${fmt(run.t, 1)} s</span><span>${t('sim.dist')} ${fmt(run.dist, 2)} m</span><span>${t('mz.crashes')}: ${run.crashed ? 1 : 0}</span>` +
    (me ? `<span>${t('mz.map_err', { e: fmt(100 * me.error, 1), c: fmt(100 * me.coverage, 0) })}</span>` : '') + '</div>';
}
let lastT = 0, acc = 0;
function loop(ts) {
  if (running) {
    const dt = robot.timing.loop_ms / 1000;
    acc += Math.min(0.1, (ts - lastT) / 1000) * +$('#rate').value;
    const n = Math.floor(acc / dt); acc -= n * dt;
    stepRun(run, n);
    if (run.found || run.crashed || run.err || run.t >= MAX_T || run.stopped) { running = false; $('#run').textContent = t('sim.run'); }
    draw(); result();
  }
  lastT = ts; requestAnimationFrame(loop);
}
$('#run').addEventListener('click', () => { if (run.found || run.crashed || run.t >= MAX_T || run.err || run.stopped) reset(); running = !running; $('#run').textContent = running ? t('sim.pause') : t('sim.run'); });
$('#reset').addEventListener('click', reset);

// edit: click an edge to toggle a wall
vReal.onPointerDown = (e) => {
  if (!$('#edit').checked) return false;
  const [x, y] = vReal.eventWorld(e), c = built.cell;
  if (st.kind === 'blocks') {  // click a square: a wall block appears / disappears (posts stay, free cells stay free)
    const b = built.blockSize, j = Math.round(x / b), i = Math.round(y / b);
    if (i % 2 === 0 && j % 2 === 1 && i > 0 && i < 2 * grid.rows && j < 2 * grid.cols) grid.H[i / 2][(j - 1) / 2] ^= 1;
    else if (i % 2 === 1 && j % 2 === 0 && j > 0 && j < 2 * grid.cols && i < 2 * grid.rows) grid.V[(i - 1) / 2][j / 2] ^= 1;
    rebuild(); return true;
  }
  const fx = x / c, fy = y / c, rx = Math.round(fx), ry = Math.round(fy);
  if (Math.abs(fy - ry) < Math.abs(fx - rx)) { const col = Math.floor(fx); if (ry > 0 && ry < grid.rows && col >= 0 && col < grid.cols) grid.H[ry][col] ^= 1; }
  else { const row = Math.floor(fy); if (rx > 0 && rx < grid.cols && row >= 0 && row < grid.rows) grid.V[row][rx] ^= 1; }
  rebuild(); return true;
};

// ---- settings ----------------------------------------------------------------------------------
function buildSide() {
  const M = $('#maze-set'); M.innerHTML = '';
  const kind = el('select', {}, el('option', { value: 'thin', text: t('mz.thin') }), el('option', { value: 'blocks', text: t('mz.blocks') })); kind.value = st.kind;
  kind.addEventListener('change', () => { st.kind = kind.value; rebuild(); buildSide(); });
  const gl = el('select', {}, el('option', { value: 'corner', text: t('mz.goal_corner') }), el('option', { value: 'center', text: t('mz.goal_center') })); gl.value = st.goal;
  gl.addEventListener('change', () => { st.goal = gl.value; rebuild(); });
  M.append(el('h3', { text: t('mz.maze_h') }), el('label', { class: 'field' }, el('span', { text: t('mz.kind') }), kind),
    slider({ key: 'mz.cols', min: 2, max: 16, step: 1, value: st.cols, onInput: (v) => { st.cols = v; newGrid(); } }),
    slider({ key: 'mz.rows', min: 2, max: 16, step: 1, value: st.rows, onInput: (v) => { st.rows = v; newGrid(); } }),
    st.kind === 'blocks'
      ? slider({ key: 'mz.block', unit: 'mm', min: Math.min(120, minCell), max: 600, step: 10, value: st.block_mm, onInput: (v) => { st.block_mm = v; rebuild(); }, hint: t('mz.block_hint', { n: minCell }) })
      : slider({ key: 'mz.cell', unit: 'mm', min: Math.min(120, minCell), max: 600, step: 10, value: st.cell_mm, onInput: (v) => { st.cell_mm = v; rebuild(); }, hint: t('mz.cell_hint', { n: minCell }) }));
  if (st.kind === 'thin') M.append(slider({ key: 'mz.wall', unit: 'mm', min: 0, max: 40, step: 1, value: st.wall_mm, onInput: (v) => { st.wall_mm = v; rebuild(); }, hint: t('mz.wall_hint') }));
  M.append(slider({ key: 'mz.braid', unit: '', min: 0, max: 0.5, step: 0.05, value: st.braid, onInput: (v) => { st.braid = v; newGrid(); }, hint: t('mz.braid_hint') }),
    el('label', { class: 'field' }, el('span', { text: t('mz.goal') }), gl),
    el('fieldset', {}, el('legend', { text: t('mz.build_err_h') }),
      slider({ key: 'mz.err_pos', unit: 'mm', min: 0, max: 10, step: 0.1, value: st.err_pos_mm, onInput: (v) => { st.err_pos_mm = v; rebuild(); } }),
      slider({ key: 'mz.err_angle', unit: '°', min: 0, max: 3, step: 0.05, value: st.err_angle_deg, onInput: (v) => { st.err_angle_deg = v; rebuild(); } }),
      el('small', { class: 'muted', text: t('mz.build_err_hint') })),
    el('div', { class: 'row' }, el('span', { text: t('mz.seed') + ': ' + st.seed }), el('button', { class: 'small', text: t('mz.new_maze'), onclick: () => { st.seed = 1 + Math.floor(Math.random() * 99999); newGrid(); buildSide(); } }),
      el('button', { class: 'small', text: t('mz.empty'), onclick: () => { grid = emptyGrid(st.rows, st.cols); rebuild(); } })));
  const A = $('#algo-set'); A.innerHTML = '';
  const as = el('select', {}, ...Object.keys(ALGORITHMS).map((k) => el('option', { value: k, text: t('algo.' + k) })), el('option', { value: 'custom', text: t('algo.custom') }));
  as.value = algoId;
  const code = el('textarea', { rows: 14, spellcheck: 'false' }); code.value = customCode; code.hidden = algoId !== 'custom';
  as.addEventListener('change', () => { algoId = as.value; localStorage.setItem('rt.maze.algo', algoId); code.hidden = algoId !== 'custom'; desc.textContent = t('algo.' + algoId + '.desc'); reset(); });
  code.addEventListener('change', () => { customCode = code.value; localStorage.setItem('rt.maze.code', customCode); reset(); });
  const desc = el('p', { class: 'muted', text: t('algo.' + algoId + '.desc') });
  A.append(el('h3', { text: t('mz.algo_h') }), as, desc, code,
    slider({ key: 'noise.global', unit: '×', min: 0, max: 3, step: 0.1, value: noise, onInput: (v) => { noise = v; reset(); } }),
    el('small', { class: 'muted', text: t('mz.algo_note') }));
  const B = $('#batch-set'); B.innerHTML = '';
  const nIn = el('input', { type: 'number', value: 10, min: 2, max: 100 });
  B.append(el('h3', { text: t('mz.batch_h') }), el('p', { class: 'muted', text: t('mz.batch_p') }), el('div', { class: 'row' }, nIn, el('button', { class: 'primary small', text: t('mz.batch_run'), onclick: () => batch(+nIn.value) })));
}

async function batch(n) {
  const box = $('#batch'); box.hidden = false;
  const rows = [];
  for (let i = 0; i < n; i++) {
    const seed = st.seed + i + 1;
    const g = generateGrid(st.rows, st.cols, seed, st.braid), b = (st.kind === 'blocks' ? constructBlocks : construct)(g, { ...st, seed }), gl = goalCells(g, st.goal);
    const R = makeRun(g, b, gl); R.goal = gl;
    while (!R.found && !R.crashed && !R.err && R.t < MAX_T && !R.stopped) { stepRun(R, 400); box.innerHTML = `<p>${t('mz.batch_prog', { i: i + 1, n })}</p>`; await new Promise((r) => setTimeout(r, 0)); }
    const me = mapError(R, g);
    rows.push({ seed, found: R.found, t: R.t, d: R.dist, crash: R.crashed, err: me ? me.error : NaN, cov: me ? me.coverage : NaN });
    R.sim.destroy();
  }
  const ok = rows.filter((r) => r.found);
  box.innerHTML = `<h3>${t('mz.batch_res', { ok: ok.length, n })}</h3><table><tr><th>${t('mz.seed')}</th><th>${t('mz.col_exit')}</th><th class="num">${t('sim.time')} (s)</th><th class="num">${t('sim.dist')} (m)</th><th>${t('mz.crashes')}</th><th class="num">${t('mz.col_map')}</th></tr>` +
    rows.map((r) => `<tr><td>${r.seed}</td><td>${r.found ? '✓' : '✗'}</td><td class="num">${fmt(r.t, 1)}</td><td class="num">${fmt(r.d, 2)}</td><td>${r.crash ? 1 : 0}</td><td class="num">${fmt(100 * r.err, 1)} % / ${fmt(100 * r.cov, 0)} %</td></tr>`).join('') +
    `<tr><th>${t('mz.mean')}</th><th>${ok.length}/${n}</th><th class="num">${fmt(ok.reduce((s, r) => s + r.t, 0) / Math.max(1, ok.length), 1)}</th><th class="num">${fmt(ok.reduce((s, r) => s + r.d, 0) / Math.max(1, ok.length), 2)}</th><th>${rows.filter((r) => r.crash).length}</th><th></th></tr></table>`;
}

buildSide();
newGrid();
requestAnimationFrame(loop);
