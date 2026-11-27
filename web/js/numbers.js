// numbers.js - "Using the Tuner's numbers in your own code": the unit table and the worked conversions are filled
// from the student's own robot profile (own-code.js), the A/B encoder drawing is generated here.
import { page, $, el, t } from './common.js';
import { loadRobot } from './store.js';
import * as O from './own-code.js';

await page();
const R = loadRobot();
const g = (x, n = 4) => String(+(+x).toPrecision(n));

// ---- units table ----
const tbl = $('#units-table');
tbl.append(el('tr', {}, el('th', { text: t('nums.th_name') }), el('th', { text: t('nums.th_value') }), el('th', { text: t('nums.th_unit') }), el('th', { text: t('nums.th_one') })));
let group = '';
for (const r of O.numberTable(R)) {
  if (r.group !== group) { group = r.group; tbl.append(el('tr', { class: 'grp' }, el('td', { colspan: 4, text: t('nums.grp_' + group) }))); }
  tbl.append(el('tr', {}, el('td', {}, el('b', { text: r.name }), el('br'), el('code', { text: r.path })),
    el('td', { class: 'num', text: g(r.value, 6) }), el('td', { text: r.unit }), el('td', { class: 'muted', text: r.one })));
}

// ---- worked conversions with this robot's numbers ----
const m = R.motor, P = m.wheel_pid_period_ms, D = R.wheels.diameter_mm, T = R.wheels.ticks_per_rev, vmax = m.max_speed_mps;
const mpt = O.metersPerTick(D, T);
const tp = Math.round(O.mpsToTp(0.1, mpt, P) * 10) / 10, v = O.tpToMps(tp, mpt, P);
const tmax = O.mpsToTp(vmax, mpt, P);
const frac = O.wheelGainsToUnit({ kp: m.wheel_kp, ki: m.wheel_ki, kd: m.wheel_kd, ff_static: m.ff_static, ff_gain: m.ff_gain }, 1 / tmax, 1 / 255);
const P2 = P * 2, g2 = O.convertWheelGains({ kp: m.wheel_kp, ki: m.wheel_ki, kd: m.wheel_kd, ff_static: m.ff_static, ff_gain: m.ff_gain }, P, P2);
const items = [
  ['nums.c_mpt', { d: D, t: g(T, 6), mpt: g(mpt * 1000) }],
  ['nums.c_tp', { tp, p: P, v: g(v) }],
  ['nums.c_mps', { v: g(v), p: P, tp }],
  ['nums.c_rpm', { v: g(v), d: D, rpm: g(O.mpsToRpm(v, D)) }],
  ['nums.c_pct', { v: g(v), vmax: g(vmax), pct: g(100 * O.mpsToFrac(v, vmax), 3) }],
  ['nums.c_gain', { kp: g(m.wheel_kp), tmax: g(tmax), kpf: g(frac.kp), ff: g(m.ff_static), fff: g(frac.ff_static) }],
  ['nums.c_period', { p: P, p2: P2, kp: g(m.wheel_kp), kp2: g(g2.kp), kd: g(m.wheel_kd), kd2: g(g2.kd), ff: g(m.ff_gain), ff2: g(g2.ff_gain), ki: g(m.wheel_ki) }],
  ['nums.c_ki', { ki: g(m.wheel_ki), p: P, kis: g(O.kiSumToKiSeconds(m.wheel_ki, P)), kd: g(m.wheel_kd), kds: g(O.kdDiffToKdSeconds(m.wheel_kd, P)) }],
];
for (const [k, vars] of items) $('#conv-list').append(el('li', { html: t(k, vars) }));

// ---- A/B signals with the counted edges for x1 / x2 / x4 ----
{
  const NS = 'http://www.w3.org/2000/svg', W = 470, x0 = 50, per = 140, H = 34;
  const svg = document.createElementNS(NS, 'svg');
  svg.setAttribute('viewBox', `0 0 ${W} 200`); svg.setAttribute('role', 'img'); svg.setAttribute('aria-label', t('nums.ab_alt'));
  const add = (tag, a) => { const e = document.createElementNS(NS, tag); for (const [k, v] of Object.entries(a)) e.setAttribute(k, v); svg.append(e); return e; };
  const text = (x, y, s, cls = '') => { const e = add('text', { x, y, class: cls }); e.textContent = s; };
  const wave = (y, shift) => {
    let d = `M${x0} ${y + H}`, hi = false;
    const edges = [];
    for (let k = 0; k < 2; k++) edges.push(x0 + shift + k * per, x0 + shift + k * per + per / 2);
    for (const e of edges) { d += ` L${e} ${hi ? y : y + H} L${e} ${hi ? y + H : y}`; hi = !hi; }
    d += ` L${x0 + 2 * per + 20} ${hi ? y : y + H}`;
    add('path', { d, class: 'ab-line' });
  };
  text(8, 30, 'A', 'ab-lab'); wave(10, 10);
  text(8, 80, 'B', 'ab-lab'); wave(60, 10 + per / 4);
  const rows = [['x1', [0]], ['x2', [0, 2]], ['x4', [0, 1, 2, 3]]];
  rows.forEach(([name, q], i) => {
    const y = 120 + i * 26;
    text(8, y + 5, name, 'ab-lab');
    for (let k = 0; k < 2; k++) for (const j of q) add('circle', { cx: x0 + 10 + k * per + j * per / 4, cy: y, r: 5, class: 'ab-tick' });
    text(x0 + 2 * per + 30, y + 5, t('nums.ab_count', { n: q.length * 2 }), 'ab-note');
  });
  $('#ab').append(svg);
}
