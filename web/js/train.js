// train.js - 4. PID TRAINING: grid search, then GA refinement (upstream method), in Web Workers.
import { page, $, el, t, download, toast, slider } from './common.js';
import { robotReady } from './simgate.js';
import { loadRobot, saveRobot, cruiseSpeed } from './store.js';
import { loadEnv } from './corridor.js';
import { grid, mutate, rngJS, trainEnv, trackSet, defaultTrainSettings, robotWidthMm } from './train-core.js';
import { TimePlot } from './plot.js';
import { cFunction, configH, modelTxt } from './export.js';
import { fmt } from './i18n.js';

await page();
robotReady();
const robot = loadRobot(), pageEnv = loadEnv();
let TS; try { TS = { ...defaultTrainSettings(), ...JSON.parse(localStorage.getItem('rt.train') || '{}') }; } catch (e) { TS = defaultTrainSettings(); }
let env = trainEnv(TS, pageEnv, robot);
function setupText() {
  env = trainEnv(TS, pageEnv, robot);
  const ws = trackSet(env).map((x) => x.width), lo = Math.min(...ws), hi = Math.max(...ws);
  $('#setup').textContent = t(TS.mode === 'width' ? 'tr.setup_width' : 'tr.setup_mine', { name: robot.name, lo, hi, a: TS.angle_deg, n: env.noise.global });
  $('#width-box').hidden = TS.mode !== 'width';
  const tight = TS.mode === 'width' && env.room_mm < 10;
  $('#setup').classList.toggle('notice', tight); $('#setup').classList.toggle('warn', tight);
  if (tight) $('#setup').textContent += ' ' + t('tr.too_narrow', { w: robotWidthMm(robot), room: env.room_mm.toFixed(0) });
  try { localStorage.setItem('rt.train', JSON.stringify(TS)); } catch (e) { /* ignore */ }
}
document.querySelectorAll('input[name="tset"]').forEach((r) => { r.checked = r.value === TS.mode; r.addEventListener('change', () => { TS.mode = r.value; setupText(); }); });
for (const [id, k] of [['#t-width', 'width_mm'], ['#t-spread', 'spread_pct'], ['#t-angle', 'angle_deg'], ['#t-cell', 'cell_mm']]) {
  const inp = $(id); inp.value = TS[k];
  inp.addEventListener('input', () => { const v = parseFloat(inp.value); if (Number.isFinite(v)) { TS[k] = v; setupText(); } });
}
setupText();
// the cruise speed is chosen here, not trained: base = v_min + speed_frac (v_max - v_min)
let speedFrac = robot.pid.speed_frac;   // the robot's own value (shared by every page)
const sfLine = el('small', { class: 'muted' });
const sfText = () => { sfLine.textContent = t('tr.speed_frac_line', { v: fmt(cruiseSpeed(robot, speedFrac), 3), vmax: fmt(robot.motor.max_speed_mps, 3), vmin: fmt(robot.motor.v_min_mps || 0, 3) }); };
$('#speed-frac').append(slider({ key: 'pg.speed_frac', min: 0, max: 1, step: 0.01, value: +speedFrac.toFixed(2), hint: t('tr.speed_frac_hint'),
  onInput: (v) => { speedFrac = Math.min(1, Math.max(0, v)); sfText(); const R2 = loadRobot(); R2.pid = { ...R2.pid, speed_frac: speedFrac }; robot.pid.speed_frac = speedFrac; saveRobot(R2); } }), sfLine);
sfText();
const fit = new TimePlot($('#plots'), { title: t('tr.plot_fit'), series: [{ name: t('tr.best'), color: '#0f6e52' }, { name: t('tr.gen_mean'), color: '#9aa6b0' }], keepAll: true, tUnit: '' });
const crash = new TimePlot($('#plots'), { title: t('tr.plot_crash'), series: [{ name: t('tr.best'), color: '#c8352e' }], keepAll: true, tUnit: '', min: 0 });

const N = Math.max(1, Math.min(8, (navigator.hardwareConcurrency || 4) - 1));
let workers = [], queue = [], busy = 0, stop = false, best = null;
const EFFORT = { quick: { eps: 2, gens: 10, pop: 12 }, normal: { eps: 4, gens: 30, pop: 20 }, thorough: { eps: 6, gens: 60, pop: 24 } };

async function initWorkers() {
  workers.forEach((w) => w.terminate());
  workers = await Promise.all(Array.from({ length: N }, () => new Promise((res) => {
    const w = new Worker(new URL('./train-worker.js', import.meta.url), { type: 'module' });
    w.onmessage = (e) => { if (e.data.type === 'ready') { w.onmessage = null; res(w); } };
    w.postMessage({ type: 'init', robot, env });
  })));
}
/** evaluate many jobs in parallel; jobs: [{cand, noise, episodes, gen, seed}] -> results in order */
function evalAll(jobs, onDone) {
  return new Promise((resolve) => {
    const out = new Array(jobs.length); let next = 0, done = 0;
    const feed = (w) => {
      if (stop || next >= jobs.length) { if (done === jobs.length || stop) resolve(out); return; }
      const id = next++;
      w.onmessage = (e) => { out[id] = { ...e.data, cand: jobs[id].cand }; done++; onDone && onDone(done, jobs.length); if (done === jobs.length) resolve(out); else feed(w); };
      w.postMessage({ type: 'eval', id, ...jobs[id] });
    };
    workers.forEach(feed);
  });
}

$('#start').addEventListener('click', async () => {
  stop = false; $('#start').disabled = true; $('#stop').disabled = false;
  const E = EFFORT[$('#effort').value], ts = speedFrac;
  const t0 = performance.now();
  $('#status').textContent = t('tr.starting', { n: N });
  await initWorkers();
  fit.clear(); crash.clear();
  const total = grid(robot, env, ts).length + E.gens * E.pop + 3 * 1;
  let doneAll = 0;
  const prog = (extra) => { const f = Math.min(1, (doneAll + extra) / total); $('#bar').style.width = (100 * f).toFixed(1) + '%';
    const el2 = (performance.now() - t0) / 1000; $('#status').textContent = t('tr.progress', { p: (100 * f).toFixed(0), s: el2.toFixed(0), r: f > 0.02 ? ((el2 / f - el2)).toFixed(0) : '…' }); };
  // 1) grid search
  const g = grid(robot, env, ts);
  let res = await evalAll(g.map((cand) => ({ cand, noise: 1, episodes: E.eps, gen: 0, seed: 1 })), (d) => prog(d));
  doneAll += g.length;
  if (stop) return finish();
  res.sort((a, b) => b.score - a.score);
  best = res[0];
  let elite = res.slice(0, 4).map((r) => r.cand);
  showBest(); fit.push(0, [best.score, res.reduce((s, r) => s + r.score, 0) / res.length]); crash.push(0, [100 * best.crash]); fit.draw(); crash.draw();
  // 2) GA refinement (elitism: the best reseeds the population with mutated copies)
  const r = rngJS(12345);
  for (let gen = 1; gen <= E.gens && !stop; gen++) {
    const pop = [best.cand, ...Array.from({ length: E.pop - 1 }, (_, i) => mutate(i < E.pop / 2 ? best.cand : elite[i % elite.length], r))];
    res = await evalAll(pop.map((cand) => ({ cand, noise: 1, episodes: E.eps, gen, seed: 1 })), (d) => prog(d));
    doneAll += pop.length;
    if (stop) break;
    // re-score the champion on this generation's corridors too (it was in pop[0])
    res.sort((a, b) => b.score - a.score);
    best = res[0];
    elite = res.slice(0, 4).map((x) => x.cand);
    fit.push(gen, [best.score, res.reduce((s, x) => s + x.score, 0) / res.length]); crash.push(gen, [100 * best.crash]); fit.draw(); crash.draw();
    showBest();
  }
  // 3) robustness report at 0x / 1x / 2x noise, more episodes, unseen seeds
  $('#status').textContent = t('tr.reporting');
  const rep = await evalAll([0, 1, 2].map((noise) => ({ cand: best.cand, noise, episodes: 6, gen: 1000 + noise, seed: 777 })));
  report(rep);
  finish();
  $('#status').textContent = t('tr.done', { s: ((performance.now() - t0) / 1000).toFixed(0) });
  $('#bar').style.width = '100%';
});
$('#stop').addEventListener('click', () => { stop = true; finish(); });
function finish() { $('#start').disabled = false; $('#stop').disabled = true; if (best) exportCode(); }

function showBest() {
  $('#best-card').hidden = false;
  const c = best.cand, v = cruiseSpeed(robot, c.speed);
  $('#best').innerHTML = [['Kp', fmt(c.kp, 4)], ['Ki', fmt(c.ki, 4)], ['Kd', fmt(c.kd, 4)], [t('pg.speed_frac'), `${fmt(c.speed, 2)} (${t('pg.speed')} ${fmt(v, 3)} m/s)`],
    [t('tr.score'), fmt(best.score, 0)], [t('tr.crash_pct'), fmt(100 * best.crash, 1) + ' %'], [t('tr.ce'), fmt(best.ce, 1) + ' mm']]
    .map(([a, b]) => `<tr><th>${a}</th><td class="num">${b}</td></tr>`).join('');
  exportCode();
}
function report(rep) {
  $('#report').hidden = false;
  $('#rep').innerHTML = `<tr><th>${t('tr.noise')}</th><th class="num">${t('tr.crash_pct')}</th><th class="num">${t('tr.finish_pct')}</th><th class="num">${t('tr.ce')}</th><th class="num">${t('tr.he')}</th></tr>` +
    rep.map((x, i) => `<tr><td>${i}×</td><td class="num">${fmt(100 * x.crash, 1)} %</td><td class="num">${fmt(100 * x.finish, 1)} %</td><td class="num">${fmt(x.ce, 1)} mm</td><td class="num">${fmt(x.he, 1)}°</td></tr>`).join('');
}
function gains() { const c = best.cand; return { kp: c.kp, ki: c.ki, kd: c.kd, speed: c.speed }; }
function exportCode() {
  if (!best) return;
  $('#export').hidden = false;
  $('#code').textContent = cFunction(robot, gains());
}
$('#dl-c').addEventListener('click', () => download('pid_gains.h', cFunction(robot, gains())));
$('#dl-cfg').addEventListener('click', () => download('config_pid.h', configH(robot, gains())));
$('#dl-model').addEventListener('click', () => download('best_pid_model.txt', modelTxt(robot, gains())));
$('#use').addEventListener('click', () => {
  const g = gains(); const R = loadRobot();
  R.pid = { ...R.pid, kp: g.kp, ki: g.ki, kd: g.kd, speed_frac: g.speed }; saveRobot(R); toast(t('pg.saved'), 'ok');
});
