// measure.js - 2. GUIDED MEASUREMENT over USB (Web Serial) with manual entry for every value.
import { page, $, el, t, toast } from './common.js';
import { loadRobot, saveRobot } from './store.js';
import { RobotLink, serialSupported } from './serial.js';
import * as P from './measure-steps.js';
import { fmt } from './i18n.js';

await page();
const link = new RobotLink();
let R = loadRobot();
const log = $('#log');
link.onLine((l) => { log.textContent += l + '\n'; log.scrollTop = log.scrollHeight; });
if (!serialSupported()) { $('#nosupport').hidden = false; $('#connect').disabled = true; $('#manual').checked = true; }
$('#connect').addEventListener('click', async () => {
  try {
    if (link.connected) { await link.disconnect(); } else { await link.connect(); }
  } catch (e) { toast(t('ms.conn_err') + ' ' + e.message, 'bad'); }
  $('#conn-state').textContent = t(link.connected ? 'ms.connected' : 'ms.disconnected');
  $('#conn-state').className = 'pill ' + (link.connected ? 'ok' : '');
  $('#connect').textContent = t(link.connected ? 'ms.disconnect' : 'ms.connect');
  render();
});
$('#manual').addEventListener('change', render);
const sendCmd = () => { const c = $('#cmd').value.trim(); if (c && link.connected) link.send(c); $('#cmd').value = ''; };
$('#send').addEventListener('click', sendCmd);
$('#cmd').addEventListener('keydown', (e) => { if (e.key === 'Enter') sendCmd(); });

function save(paths) { for (const p of paths) R.source[p] = 'MEASURED'; saveRobot(R); toast(t('ms.saved'), 'ok'); render(); }
const tofNames = () => R.sensors.map((s, i) => (s.name || ['right', 'front', 'left'][i] || 's' + i).toLowerCase());

// ---- step definitions: id, fields shown/typed {label, get, set, unit}, run(ui) ------------------------
const STEPS = [
  { id: 'physical', physical: true },
  { id: 'ticks', fields: [['ms.f.ticks', () => R.wheels.ticks_per_rev, (v) => { R.wheels.ticks_per_rev = v; }, '', 'wheels.ticks_per_rev']],
    async run(ui) {
      await link.send('CAL TICKS');
      ui.ask(t('ms.ticks.ask'), t('ms.ticks.done_btn'));
      await ui.answered();
      await link.send('CAL END');
      const v = await link.waitFor(P.parseTicks, 5000);
      ui.result(v, 'ticks', [['ms.f.ticks', v.ticks]], () => { R.wheels.ticks_per_rev = v.ticks; save(['wheels.ticks_per_rev']); });
    } },
  { id: 'maxspeed', fields: [['ms.f.maxspeed', () => R.motor.max_speed_mps, (v) => { R.motor.max_speed_mps = v; }, 'm/s', 'motor.max_speed_mps']],
    async run(ui) {
      await link.send('CAL MAXSPEED');
      const v = await link.waitFor(P.parseMaxSpeed, 8000);
      ui.result(v, 'maxspeed', [['ms.f.maxspeed', v.max, 'm/s']], () => {
        const old = R.motor.max_speed_mps; R.motor.max_speed_mps = v.max;
        if (R.pid.speed_mps > v.max || R.pid.speed_mps === old) R.pid.speed_mps = v.max * 0.8;
        save(['motor.max_speed_mps']);
      });
    } },
  { id: 'ff', fields: [['ms.f.ff_static', () => R.motor.ff_static, (v) => { R.motor.ff_static = v; }, 'PWM', 'motor.ff_static'], ['ms.f.ff_gain', () => R.motor.ff_gain, (v) => { R.motor.ff_gain = v; }, 'PWM/tick', 'motor.ff_gain']],
    async run(ui) {
      await link.send('CAL FF');
      const lines = await link.collect((l) => /FF sweep done|CAL: FF/.test(l), 120000);
      const sug = Object.fromEntries(lines.map(P.parseSuggestion).filter(Boolean).map((s) => [s.key, s.value]));
      ui.result(sug, null, [['ms.f.ff_static', sug.ff_static, 'PWM'], ['ms.f.ff_gain', sug.ff_gain, 'PWM/tick']], () => {
        if (sug.ff_static !== undefined) R.motor.ff_static = sug.ff_static; if (sug.ff_gain !== undefined) R.motor.ff_gain = sug.ff_gain;
        save(['motor.ff_static', 'motor.ff_gain']);
      });
    } },
  { id: 'step', fields: [['ms.f.tau', () => R.motor.step_tau_s, (v) => { R.motor.step_tau_s = v; }, 's', 'motor.step_tau_s']],
    async run(ui) {
      await link.send('CAL STEP');
      const v = await link.waitFor(P.parseStep, 15000);
      ui.result(v, 'step', [['ms.f.tau', v.tau, 's'], ['ms.f.rise', (v.riseL + v.riseR) / 2, 's'], ['ms.f.over', (v.overL + v.overR) / 2, '%']], () => { R.motor.step_tau_s = +v.tau.toFixed(4); save(['motor.step_tau_s']); });
    } },
  { id: 'drive', fields: [['ms.f.scale_l', () => R.wheels.scale_l, (v) => { R.wheels.scale_l = v; }, '', 'wheels.scale_l'], ['ms.f.scale_r', () => R.wheels.scale_r, (v) => { R.wheels.scale_r = v; }, '', 'wheels.scale_r']],
    async run(ui) {
      await link.send('CAL DRIVE 1');
      await link.waitFor(P.parseDriveDone, 60000);
      const d = await ui.askNumber(t('ms.drive.ask'), 'm');
      await link.send(`CAL DIST ${d}`);
      const v = await link.waitFor(P.parseDist, 5000);
      ui.result(v, null, [['ms.f.scale_l', v.scale_l], ['ms.f.scale_r', v.scale_r]], () => { R.wheels.scale_l = v.scale_l; R.wheels.scale_r = v.scale_r; save(['wheels.scale_l', 'wheels.scale_r']); });
    } },
  { id: 'spin', fields: [['ms.f.track', () => R.wheels.track_effective_mm, (v) => { R.wheels.track_effective_mm = v; }, 'mm', 'wheels.track_effective_mm']],
    async run(ui) {
      await link.send('CAL SPIN 5');
      const s = await link.waitFor(P.parseSpinDone, 120000);
      if (!s.trusted) ui.warn(t('ms.spin.untrusted'));
      const n = await ui.askNumber(t('ms.spin.ask'), t('ms.turns'));
      await link.send(`CAL TURNS ${n}`);
      const v = await link.waitFor(P.parseTurns, 5000);
      ui.result(v, 'spin', [['ms.f.track', v.track_m * 1000, 'mm']], () => { R.wheels.track_effective_mm = +(v.track_m * 1000).toFixed(1); save(['wheels.track_effective_mm']); });
    } },
  { id: 'tof', get fields() { return R.sensors.flatMap((s, i) => [[`#${i + 1} ${t('ms.f.noise')}`, () => s.noise_mm, (v) => { s.noise_mm = v; }, 'mm', `sensors.${i}.noise_mm`], [`#${i + 1} ${t('ms.f.bias')}`, () => s.bias_mm, (v) => { s.bias_mm = v; }, 'mm', `sensors.${i}.bias_mm`]]); },
    async run(ui) {
      const d = await ui.askNumber(t('ms.tof.ask'), 'mm');
      await link.send(`CAL TOF 200 ${d}`);
      const lines = await link.collect((l) => /TOF done/.test(l), 70000);
      const stats = lines.map(P.parseTof).filter(Boolean), bias = lines.map(P.parseTofBias).filter(Boolean);
      const names = tofNames();
      const fields = [];
      for (const s of stats) {
        const i = names.indexOf(s.name.toLowerCase()); if (i < 0) continue;
        const b = bias.find((x) => x.name === s.name);
        fields.push([`#${i + 1} ${s.name}: ${t('ms.f.noise')}`, s.std, 'mm'], [`#${i + 1} ${s.name}: ${t('ms.f.bias')}`, b ? b.bias : 0, 'mm'], [`#${i + 1} ${s.name}: ${t('ms.f.drop')}`, s.dropPct, '%']);
        const c = P.sanity('tof', s, R); if (c) ui.warn(t(c[1], c[2]));
      }
      ui.result({}, null, fields, () => {
        const paths = [];
        for (const s of stats) {
          const i = names.indexOf(s.name.toLowerCase()); if (i < 0) continue;
          const b = bias.find((x) => x.name === s.name);
          R.sensors[i].noise_mm = +Math.max(0.5, s.std).toFixed(2); R.sensors[i].dropout_pct = s.dropPct; if (b) R.sensors[i].bias_mm = b.bias;
          paths.push(`sensors.${i}.noise_mm`, `sensors.${i}.bias_mm`);
        }
        save(paths);
      });
    } },
  { id: 'timing', fields: [['ms.f.loop', () => R.timing.loop_ms, (v) => { R.timing.loop_ms = v; }, 'ms', 'timing.loop_ms'], ['ms.f.age', () => R.timing.age_ms, (v) => { R.timing.age_ms = v; }, 'ms', 'timing.age_ms'], ['ms.f.age_j', () => R.timing.age_jitter_ms, (v) => { R.timing.age_jitter_ms = v; }, 'ms', 'timing.age_jitter_ms']],
    async run(ui) {
      await link.send('LOAD SENSORS');
      await new Promise((r) => setTimeout(r, 1500));
      const per = [], ages = [];
      for (let k = 0; k < 12; k++) {
        const lp = link.waitFor(P.parseStatusLoop, 3000), ag = link.waitFor(P.parseStatusAges, 3000);
        await link.send('STATUS');
        try { per.push((await lp).period_ms); ages.push((await ag).ages_ms); } catch (e) { /* missed one */ }
        ui.progress((k + 1) / 12);
        await new Promise((r) => setTimeout(r, 300));
      }
      await link.send('LOAD FINAL');
      if (!per.length) throw new Error(t('ms.timing.none'));
      const v = P.summarizeTiming(per, ages.length ? ages : [[R.timing.age_ms]]);
      ui.result(v, 'timing', [['ms.f.loop', v.loop_ms, 'ms'], ['ms.f.age', v.age_ms, 'ms'], ['ms.f.age_j', v.age_jitter_ms, 'ms']], () => { Object.assign(R.timing, v); save(['timing.loop_ms', 'timing.age_ms', 'timing.age_jitter_ms']); });
    } },
];

const open = new Set(['physical']);
function render() {
  R = loadRobot();
  const box = $('#steps'); box.innerHTML = '';
  STEPS.forEach((s, n) => box.append(stepCard(s, n + 1)));
}
function stepCard(s, n) {
  const k = 'ms.' + s.id;
  const done = s.fields && s.fields.every((f) => R.source[f[4]] === 'MEASURED');
  const card = el('section', { class: 'step' + (open.has(s.id) ? ' open' : '') + (done ? ' done' : '') });
  const head = el('header', {}, el('span', { class: 'num', text: done ? '✓' : String(n) }), el('b', { text: t(k + '.title') }), el('span', { class: 'spacer' }),
    done ? el('span', { class: 'pill ok', text: t('ms.measured') }) : null);
  head.addEventListener('click', () => { open.has(s.id) ? open.delete(s.id) : open.add(s.id); card.classList.toggle('open'); });
  const body = el('div', { class: 'body' });
  body.append(el('dl', {}, el('dt', { text: t('ms.what') }), el('dd', { text: t(k + '.what') }), el('dt', { text: t('ms.why') }), el('dd', { text: t(k + '.why') }),
    el('dt', { text: t('ms.how') }), el('dd', { text: t(k + '.how') })));
  body.append(el('div', { class: 'media', 'data-slot': `img/measure/${s.id}.gif`, text: t('ms.media', { f: `img/measure/${s.id}.gif` }) }));
  if (s.physical) {
    body.append(el('p', { text: t('ms.physical.p') }), el('a', { class: 'btn', href: 'builder.html', text: t('ms.physical.btn') }));
    card.append(head, body); return card;
  }
  const out = el('div', { class: 'stack' });
  const manual = $('#manual').checked || !link.connected;
  const startBtn = el('button', { class: 'primary', text: t('ms.start'), disabled: !link.connected ? true : null });
  const ui = makeUI(out, s);
  startBtn.addEventListener('click', async () => {
    startBtn.disabled = true; out.innerHTML = '';
    try { await s.run(ui); } catch (e) { out.append(el('p', { class: 'notice bad', text: t('ms.failed') + ' ' + e.message })); }
    startBtn.disabled = false;
  });
  body.append(el('div', { class: 'row' }, startBtn, link.connected ? null : el('span', { class: 'muted', text: t('ms.connect_first') })), out);
  if (manual) {
    const man = el('div', { class: 'card stack' }, el('b', { text: t('ms.type_h') }));
    for (const [lab, get, set, unit, path] of s.fields) {
      const inp = el('input', { type: 'number', step: 'any', value: get() });
      man.append(el('label', { class: 'field' }, el('span', { text: (lab.startsWith('ms.') ? t(lab) : lab) + (unit ? ` (${unit})` : '') }), inp));
      inp.addEventListener('change', () => { const v = parseFloat(inp.value); if (Number.isFinite(v)) { set(v); R.source[path] = 'MEASURED'; saveRobot(R); } });
    }
    man.append(el('small', { class: 'muted', text: t('ms.type_note') }));
    body.append(man);
  }
  card.append(head, body);
  return card;
}
function makeUI(out, s) {
  let resolveAns = null;
  return {
    ask(text, btn) { out.append(el('p', { class: 'notice', text }), el('button', { text: btn, onclick: () => resolveAns && resolveAns() })); },
    answered() { return new Promise((r) => { resolveAns = r; }); },
    askNumber(text, unit) {
      return new Promise((res) => {
        const inp = el('input', { type: 'number', step: 'any' });
        out.append(el('p', { class: 'notice', text }), el('div', { class: 'row' }, inp, el('span', { text: unit }), el('button', { class: 'primary', text: t('ms.ok'), onclick: () => { const v = parseFloat(inp.value); if (Number.isFinite(v)) res(v); } })));
      });
    },
    warn(text) { out.append(el('p', { class: 'notice warn', text })); },
    progress(f) { let b = out.querySelector('.progress'); if (!b) { b = el('div', { class: 'progress' }, el('div')); out.append(b); } b.firstChild.style.width = (100 * f) + '%'; },
    result(v, check, rows, apply) {
      const c = check ? P.sanity(check, v, R) : null;
      out.append(el('table', {}, ...rows.map(([lab, val, unit]) => el('tr', {}, el('th', { text: lab.startsWith('ms.') ? t(lab) : lab }), el('td', { class: 'num', text: `${fmt(val, 4)} ${unit || ''}` })))));
      if (c) out.append(el('p', { class: 'notice ' + (c[0] === 'bad' ? 'bad' : 'warn'), text: t(c[1], c[2]) }));
      else out.append(el('p', { class: 'pill ok', text: t('ms.plausible') }));
      out.append(el('button', { class: 'primary', text: t('ms.apply'), onclick: apply }));
    },
  };
}
render();
