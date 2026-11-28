// builder.js - 1. ROBOT BUILDER: draw + edit the robot, sensor library, JSON / robot_hardware.txt.
import { page, $, $$, el, t, download, pickFile, toast } from './common.js';
import { loadRobot, saveRobot, defaultRobot, makeSensor, migrate, SENSOR_LIBRARY, SENSOR_FIELDS, robotToHardwareTxt, hardwareTxtToRobot, downloadName } from './store.js';
import { View, robotShapes, css, PALETTE } from './draw.js';
import { usesBt, btName, BOARDS, pinLabel, pinTags, ENCODERS, DRIVERS, generateSketch, validateHardware, hasErrors, suggestPins, computedTicksPerRev, sketchTicks,
  isLowRes, edgesPerPulse, requiredLibraries, sensorWireName, normalizeHardware, defaultHardware, unoHardware } from './firmware-gen.js';
import { choiceBar, mountOwn, isOwnMode } from './own-code-ui.js';
import { robotToFile, fileToRobot, isRobotFile, allFields } from './robotfile.js';
import { isSet } from './store.js';

await page();
let R = loadRobot();
let selected = -1;  // selected sensor
const view = new View($('#cv'), { scale: 1800, aspect: 0.75, minH: 360, maxH: 680 });

function commit(redrawForm = false) {
  saveRobot(R);
  if (redrawForm) buildForms();
  draw(); checks(); fwChecks();
  $('#json').textContent = JSON.stringify(R, null, 1);
}
function mark(path, tag = 'TYPED') { R.source[path] = tag; }

// ---- numeric field bound to R[group][key] -------------------------------------------------------
function num(group, key, labelKey, unit, opts = {}) {
  const obj = typeof group === 'string' ? R[group] : group;
  const path = opts.path || `${group}.${key}`;
  const full = typeof group === 'string' ? `${group}.${key}` : `${opts.path}.${key}`;   // robot-file path of this value
  const set = isSet(R, full);
  const inp = el('input', { type: 'number', step: opts.step || 'any', value: set ? obj[key] : '', placeholder: 'N/A', min: opts.min, 'data-path': full });
  inp.addEventListener('input', () => {
    const v = parseFloat(inp.value);
    if (!Number.isFinite(v)) return;
    obj[key] = v; mark(path); src.textContent = 'TYPED'; src.className = 'pill src'; inp.classList.remove('unset'); commit();
  });
  if (!set) inp.classList.add('unset');
  const tag = set ? (R.source[path] || (R.all_set ? 'SET' : 'DATASHEET')) : 'N/A';
  const src = el('span', { class: 'pill src ' + (tag === 'MEASURED' ? 'ok' : tag === 'N/A' ? 'warn' : ''), text: tag, title: t('builder.source_hint') });
  return el('label', { class: 'field' }, el('span', {}, el('span', { text: t(labelKey) }), unit ? el('span', { class: 'muted', text: ` (${unit})` }) : null, ' ', src), inp);
}

function buildForms() {
  $('#robot-name').value = R.name;
  $('#drive').value = R.drive;
  // geometry
  const g = $('[data-panel="geom"]'); g.innerHTML = '';
  g.append(el('div', { class: 'card stack' }, el('h3', { text: t('builder.body_h') }),
    num('body', 'front_mm', 'builder.front', 'mm'), num('body', 'rear_mm', 'builder.rear', 'mm'), num('body', 'width_mm', 'builder.width', 'mm'),
    el('small', { class: 'muted', text: t('builder.origin_note') })));
  const wheels = el('div', { class: 'card stack' }, el('h3', { text: t('builder.wheels_h') }),
    num('wheels', 'diameter_mm', 'builder.wheel_d', 'mm'), num('wheels', 'width_mm', 'builder.wheel_w', 'mm'),
    num('wheels', 'track_mm', 'builder.track', 'mm'), num('wheels', 'x_mm', 'builder.axle_x', 'mm'));
  if (R.drive === '4wd') wheels.append(num('wheels', 'wheelbase_mm', 'builder.wheelbase', 'mm'), el('small', { class: 'muted', text: t('builder.skid_note') }));
  wheels.append(num('wheels', 'track_effective_mm', 'builder.track_eff', 'mm'), el('small', { class: 'muted', text: t('builder.track_eff_note') }));
  g.append(wheels);
  const cas = el('div', { class: 'card stack' }, el('h3', { text: t('builder.caster_h') }));
  const cb = el('input', { type: 'checkbox' }); cb.checked = R.caster.enabled;
  cb.addEventListener('change', () => { R.caster.enabled = cb.checked; mark('caster.enabled'); commit(); });
  cas.append(el('label', { class: 'toggle' }, cb, el('span', { text: t('builder.caster_on') })), num('caster', 'x_mm', 'builder.caster_x', 'mm'), num('caster', 'y_mm', 'builder.caster_y', 'mm'), num('caster', 'diameter_mm', 'builder.caster_d', 'mm'),
    el('small', { class: 'muted', text: t('builder.caster_note') }));
  if (R.drive === '2wd') g.append(cas);
  // sensors
  const s = $('[data-panel="sensors"]'); s.innerHTML = '';
  s.append(el('p', { class: 'muted', text: t('builder.sensors_intro') }));
  R.sensors.forEach((sen, i) => s.append(sensorCard(sen, i)));
  const add = el('button', { text: t('builder.add_sensor'), disabled: R.sensors.length >= 5 ? true : null, onclick: () => {
    R.sensors.push(makeSensor(R.sensors[0] ? R.sensors[0].type : 'VL53L1X_short', { x_mm: R.body.front_mm - 10, angle_deg: 0, name: 's' + R.sensors.length }));
    R.hardware.sensors.push({ xshut: -1, addr: 0x30 + R.sensors.length - 1, trig: -1, echo: -1, analog: -1 });
    selected = R.sensors.length - 1; commit(true);
  } });
  s.append(el('div', { class: 'row' }, add, el('span', { class: 'muted', text: t('builder.max5') })));
  // motor + timing
  const m = $('[data-panel="motor"]'); m.innerHTML = '';
  m.append(el('div', { class: 'card stack' }, el('h3', { text: t('builder.drive_h') }),
    num('wheels', 'ticks_per_rev', 'builder.ticks', t('unit.ticks')), num('motor', 'max_speed_mps', 'builder.max_speed', 'm/s'), num('motor', 'v_min_mps', 'builder.v_min', 'm/s'),
    num('motor', 'step_tau_s', 'builder.step_tau', 's'), el('small', { class: 'muted', text: t('builder.step_tau_note') }),
    num('motor', 'speed_tau_ms', 'builder.speed_tau', 'ms', { min: 0 }), el('small', { class: 'muted', html: t('builder.speed_tau_note') })));
  m.append(el('div', { class: 'card stack' }, el('h3', { text: t('builder.timing_h') }),
    num('timing', 'loop_ms', 'builder.loop_ms', 'ms'), num('timing', 'age_ms', 'builder.age_ms', 'ms'), num('timing', 'age_jitter_ms', 'builder.age_jitter', 'ms'),
    el('small', { class: 'muted', text: t('builder.timing_note') })));
  const sel = el('select', {}, el('option', { value: 'pid', text: t('builder.model_pid') }), el('option', { value: 'lag', text: t('builder.model_lag') }));
  sel.value = R.motor.model;
  sel.addEventListener('change', () => { R.motor.model = sel.value; commit(true); });
  const adv = el('details', { class: 'card' }, el('summary', { text: t('builder.motor_adv') }), el('div', { class: 'stack' },
    el('label', { class: 'field' }, el('span', { text: t('builder.motor_model') }), sel), el('small', { class: 'muted', text: t('builder.motor_model_note') })));
  if (R.motor.model === 'pid') {
    const st = adv.querySelector('.stack');
    for (const [k, u] of [['ff_static', 'PWM'], ['ff_gain', 'PWM/tick'], ['wheel_kp', ''], ['wheel_ki', ''], ['wheel_kd', ''], ['wheel_pid_period_ms', 'ms'],
      ['pwm_max', ''], ['pwm_limit', ''], ['k_l', 'm/s/PWM'], ['k_r', 'm/s/PWM'], ['static_l', 'PWM'], ['static_r', 'PWM'], ['breakaway_l', 'PWM'], ['breakaway_r', 'PWM'],
      ['tau_l_s', 's'], ['tau_r_s', 's'], ['battery_v', 'V'], ['nominal_v', 'V']]) st.append(num('motor', k, 'motor.' + k, u));
  }
  m.append(adv);
  buildFirmware();
}

function sensorCard(sen, i) {
  const card = el('div', { class: 'card stack' + (i === selected ? ' selected' : ''), style: `border-inline-start:4px solid ${PALETTE[i % PALETTE.length]}` });
  const type = el('select', {}, ...Object.keys(SENSOR_LIBRARY).map((k) => el('option', { value: k, text: t('sensor.' + k) })));
  type.value = sen.type;
  type.addEventListener('change', () => {
    mark(`sensors.${i}.type`);
    const fresh = makeSensor(type.value, sen);
    Object.assign(sen, fresh, { x_mm: sen.x_mm, y_mm: sen.y_mm, angle_deg: sen.angle_deg, name: sen.name });
    commit(true);
  });
  const name = el('input', { value: sen.name || '', style: 'width:90px' });
  name.addEventListener('input', () => { sen.name = name.value; mark(`sensors.${i}`); commit(); });
  const rm = el('button', { class: 'small', text: '✕', title: t('builder.remove'), onclick: () => { R.sensors.splice(i, 1); R.hardware.sensors.splice(i, 1); selected = -1; commit(true); } });
  if (R.sensors.length <= 1) rm.disabled = true;
  card.append(el('div', { class: 'row' }, el('b', { text: `#${i + 1}` }), name, type, el('span', { class: 'spacer' }), rm));
  card.append(el('small', { class: 'muted', text: t(SENSOR_LIBRARY[sen.type].note) }));
  card.append(num(sen, 'x_mm', 'builder.s_x', 'mm', { path: `sensors.${i}` }), num(sen, 'y_mm', 'builder.s_y', 'mm', { path: `sensors.${i}` }),
    num(sen, 'angle_deg', 'builder.s_angle', '°', { path: `sensors.${i}` }));
  const det = el('details', {}, el('summary', { text: t('builder.datasheet') }));
  const units = { fov_deg: '°', min_mm: 'mm', max_mm: 'mm', period_ms: 'ms', noise_mm: 'mm', noise_pct: '%', dropout_pct: '%', rays: '', max_incidence_deg: '°', bias_mm: 'mm' };
  for (const f of SENSOR_FIELDS) det.append(num(sen, f, 'sensor.f.' + f, units[f], { path: `sensors.${i}.${f}` }));
  det.append(el('button', { class: 'small', text: t('builder.reset_ds'), onclick: () => {
    const d = makeSensor(sen.type); for (const f of SENSOR_FIELDS) if (f !== 'bias_mm') sen[f] = d[f]; commit(true);
  } }));
  card.append(det);
  card.addEventListener('click', () => { if (selected !== i) { selected = i; draw(); } });
  return card;
}

// ---- drawing -------------------------------------------------------------------------------------
function draw() {
  const g = view.begin();
  view.grid(0.01);
  const S = robotShapes(R), m = (p) => [p[0] / 1000, p[1] / 1000];
  // sensor cones (to 25 cm or the max range)
  S.sensors.forEach((s, i) => {
    const a = s.angle_deg * Math.PI / 180, h = s.fov_deg / 2 * Math.PI / 180, L = Math.min(0.25, s.max_mm / 1000);
    const x = s.x_mm / 1000, y = s.y_mm / 1000, c = PALETTE[i % PALETTE.length];
    view.poly([[x, y], [x + Math.cos(a - h) * L, y + Math.sin(a - h) * L], [x + Math.cos(a + h) * L, y + Math.sin(a + h) * L]], c + (i === selected ? '40' : '20'), null);
    view.polyline([[x, y], [x + Math.cos(a) * L, y + Math.sin(a) * L]], c, i === selected ? 2.5 : 1.5, [4, 3]);
  });
  S.wheels.forEach((w) => view.poly(w.map(m), '#2b3442', null, 0.85));
  view.poly(S.body.map(m), css('--robot') + '30', css('--robot'));
  if (S.caster) { view.dot(S.caster.x / 1000, S.caster.y / 1000, Math.max(3, S.caster.d / 2000 * view.scale), '#888'); }
  S.sensors.forEach((s, i) => {
    const x = s.x_mm / 1000, y = s.y_mm / 1000, a = s.angle_deg * Math.PI / 180, c = PALETTE[i % PALETTE.length];
    const d = 0.006, w = 0.004;
    const pts = [[-w, -d], [w, -d], [w, d], [-w, d]].map(([u, v]) => [x + u * Math.cos(a) - v * Math.sin(a), y + u * Math.sin(a) + v * Math.cos(a)]);
    view.poly(pts, c, i === selected ? css('--text') : null);
    view.text(x + Math.cos(a) * 0.02, y + Math.sin(a) * 0.02 + 0.004, `#${i + 1}`, c, 'center');
  });
  // origin + axes
  view.dot(0, 0, 4, css('--bad'));
  view.polyline([[0, 0], [0.04, 0]], css('--bad'), 1.5); view.polyline([[0, 0], [0, 0.04]], css('--ok'), 1.5);
  view.text(0.042, -0.002, '+x', css('--bad')); view.text(0.002, 0.044, '+y', css('--ok'));
  // dimensions
  const b = R.body;
  dim([b.front_mm / 1000, -b.width_mm / 2000 - 0.02], [-b.rear_mm / 1000, -b.width_mm / 2000 - 0.02], `${b.front_mm + b.rear_mm} mm`);
  dim([-b.rear_mm / 1000 - 0.02, b.width_mm / 2000], [-b.rear_mm / 1000 - 0.02, -b.width_mm / 2000], `${b.width_mm} mm`);
  const tw = R.wheels.track_mm / 1000, wx = R.wheels.x_mm / 1000;
  dim([wx + 0.035, tw / 2], [wx + 0.035, -tw / 2], `${t('builder.track_short')} ${R.wheels.track_mm} mm`);
  view.scaleBar();
  if (!view.userMoved) { /* keep the robot centred */ }
}
function dim(a, b, label) {
  view.polyline([a, b], css('--muted'), 1, [2, 3]);
  view.text((a[0] + b[0]) / 2, (a[1] + b[1]) / 2 - 0.006, label, css('--muted'), 'center');
}
view.onChange = draw;
function fitRobot() {
  const b = R.body, ext = Math.max(b.front_mm, b.rear_mm, b.width_mm / 2, R.wheels.track_mm / 2 + R.wheels.width_mm) / 1000;
  view.fit(-ext - 0.06, -ext - 0.06, ext + 0.12, ext + 0.06, 0.05);
}

// drag a sensor on the canvas (Shift + drag = rotate it)
let drag = null;
view.onPointerDown = (e) => {
  const [x, y] = view.eventWorld(e);
  let best = -1, bd = 0.012;
  if (!e.altKey) R.sensors.forEach((s, i) => { const d = Math.hypot(s.x_mm / 1000 - x, s.y_mm / 1000 - y); if (d < bd) { bd = d; best = i; } });
  if (best < 0) {   // no sensor there (or Alt held): the caster ball
    const c = R.caster;   // the caster ball can be dragged too
    if (!c.enabled || R.drive !== '2wd' || Math.hypot(c.x_mm / 1000 - x, (c.y_mm || 0) / 1000 - y) > Math.max(0.012, c.diameter_mm / 2000)) return false;
    drag = { caster: true }; view.c.setPointerCapture(e.pointerId); return true;
  }
  selected = best; drag = { i: best, rotate: e.shiftKey }; view.c.setPointerCapture(e.pointerId); return true;
};
view.onPointerMove = (e) => {
  if (!drag) return false;
  if (drag.caster) {
    const [x, y] = view.eventWorld(e);
    R.caster.x_mm = Math.round(x * 2000) / 2; R.caster.y_mm = Math.round(y * 2000) / 2;
    R.source['caster.x_mm'] = R.source['caster.y_mm'] = 'TYPED';
    draw(); return true;
  }
  const [x, y] = view.eventWorld(e), s = R.sensors[drag.i];
  if (drag.rotate) s.angle_deg = Math.round(Math.atan2(y - s.y_mm / 1000, x - s.x_mm / 1000) * 180 / Math.PI);
  else { s.x_mm = Math.round(x * 2000) / 2; s.y_mm = Math.round(y * 2000) / 2; }
  R.source[`sensors.${drag.i}`] = 'TYPED';
  draw(); return true;
};
view.onPointerUp = () => { if (drag) { drag = null; commit(true); } };

// ---- checks ("sanity") ---------------------------------------------------------------------------
function checks() {
  const out = [];
  const side = R.sensors.filter((s) => Math.abs(s.angle_deg) >= 15);
  const left = R.sensors.some((s) => s.angle_deg >= 15), right = R.sensors.some((s) => s.angle_deg <= -15);
  if (!left && !right) out.push(['bad', t('builder.chk_noside')]);
  else if (!(left && right)) out.push(['warn', t('builder.chk_oneside')]);
  else out.push(['ok', t('builder.chk_twoside')]);
  if (R.wheels.track_mm > R.body.width_mm + 2 * R.wheels.width_mm) out.push(['warn', t('builder.chk_wheels_out')]);
  if (Math.abs(R.wheels.track_effective_mm - R.wheels.track_mm) > 0.25 * R.wheels.track_mm) out.push(['warn', t('builder.chk_track_eff')]);
  R.sensors.forEach((s, i) => {
    if (s.x_mm > R.body.front_mm + 15 || s.x_mm < -R.body.rear_mm - 15 || Math.abs(s.y_mm) > R.body.width_mm / 2 + 15) out.push(['warn', t('builder.chk_sensor_out', { n: i + 1 })]);
  });
  if (R.drive === '4wd' && R.wheels.wheelbase_mm < R.wheels.diameter_mm) out.push(['warn', t('builder.chk_wheelbase')]);
  if (side.length && Math.min(...side.map((s) => s.max_mm)) < 150) out.push(['warn', t('builder.chk_range')]);
  $('#checks').innerHTML = '';
  $('#checks').append(el('h3', { text: t('builder.checks_h') }), ...out.map(([k, m]) => el('p', {}, el('span', { class: 'pill ' + k, text: t('chk.' + k) }), ' ', m)));
}

// ---- tabs / header fields / file buttons -----------------------------------------------------------
$$('[role="tab"]').forEach((b) => b.addEventListener('click', () => {
  $$('[role="tab"]').forEach((x) => x.setAttribute('aria-selected', x === b));
  $$('[data-panel]').forEach((p) => { p.hidden = p.dataset.panel !== b.dataset.tab; });
  const tabs = $('.tabs'); if (tabs.getBoundingClientRect().top < 80) tabs.scrollIntoView({ block: 'start' });   // the new tab starts at its top
}));
$('#robot-name').addEventListener('input', (e) => { R.name = e.target.value; mark('name'); commit(); });
$('#drive').addEventListener('change', (e) => { R.drive = e.target.value; mark('drive'); if (R.drive === '4wd') R.caster.enabled = false; commit(true); fitRobot(); draw(); });
$('#exp-file').addEventListener('click', () => download(downloadName(R, '.robot.txt'), robotToFile(R)));
$('#imp-file').addEventListener('click', async () => {
  const f = await pickFile('.txt,.json,text/plain,application/json'); if (!f) return;
  try {
    if (/^\s*\{/.test(f.text)) R = migrate(JSON.parse(f.text));
    else if (isRobotFile(f.text)) { const res = fileToRobot(f.text); R = res.robot; if (res.unknown.length) toast(t('builder.file_unknown', { keys: res.unknown.join(', ') }), 'warn'); }
    else R = hardwareTxtToRobot(f.text, R);
    commit(true); fitRobot(); draw(); toast(t('builder.imported', { name: f.name }), 'ok');
  } catch (e) { toast(t('builder.import_err') + ' ' + e.message, 'bad'); }
});
$('#exp-json').addEventListener('click', () => download(downloadName(R, '.robot.json'), JSON.stringify(R, null, 2), 'application/json'));
$('#imp-json').addEventListener('click', async () => {
  const f = await pickFile('.json,application/json'); if (!f) return;
  try { R = migrate(JSON.parse(f.text)); commit(true); fitRobot(); draw(); toast(t('builder.imported', { name: f.name }), 'ok'); } catch (e) { toast(t('builder.import_err') + ' ' + e.message, 'bad'); }
});
$('#exp-hw').addEventListener('click', () => download('robot_hardware.txt', robotToHardwareTxt(R)));
$('#imp-hw').addEventListener('click', async () => {
  const f = await pickFile('.txt,text/plain'); if (!f) return;
  try { R = hardwareTxtToRobot(f.text, R); commit(true); fitRobot(); draw(); toast(t('builder.imported', { name: f.name }), 'ok'); } catch (e) { toast(t('builder.import_err') + ' ' + e.message, 'bad'); }
});
function blank(drive) {
  const r = defaultRobot();
  r.name = t('builder.new_name'); r.drive = drive; r.source = {};
  r.body = { front_mm: 120, rear_mm: 60, width_mm: 150 };
  r.wheels = { ...r.wheels, diameter_mm: 65, width_mm: 26, track_mm: 140, x_mm: 0, wheelbase_mm: 100, track_effective_mm: drive === '4wd' ? 190 : 140, ticks_per_rev: 1440, scale_l: 1, scale_r: 1 };
  r.caster = { enabled: drive === '2wd', x_mm: 90, y_mm: 0, diameter_mm: 15 };
  r.motor = { ...r.motor, model: 'lag', max_speed_mps: 0.5, step_tau_s: 0.12 };
  r.sensors = [makeSensor('VL53L0X', { name: 'right', x_mm: 110, y_mm: -40, angle_deg: -45 }), makeSensor('VL53L0X', { name: 'front', x_mm: 120, y_mm: 0, angle_deg: 0 }),
    makeSensor('VL53L0X', { name: 'left', x_mm: 110, y_mm: 40, angle_deg: 45 })];
  r.timing = { loop_ms: 33, age_ms: 25, age_jitter_ms: 15 };
  r.pid = { ...r.pid, kp: 1.0, ki: 0.0, kd: 0.1, speed_frac: 0.75 };
  r.hardware = unoHardware();
  return r;
}
$('#new-2wd').addEventListener('click', () => { if (confirm(t('builder.confirm_new'))) { R = blank('2wd'); commit(true); fitRobot(); draw(); } });
$('#new-4wd').addEventListener('click', () => { if (confirm(t('builder.confirm_new'))) { R = blank('4wd'); commit(true); fitRobot(); draw(); } });
$('#reset-example').addEventListener('click', () => { if (confirm(t('builder.confirm_new'))) { R = defaultRobot(); commit(true); fitRobot(); draw(); } });

function showTab(id) {
  $$('[role="tab"]').forEach((x) => x.setAttribute('aria-selected', x.dataset.tab === id));
  $$('[data-panel]').forEach((p) => { p.hidden = p.dataset.panel !== id; });
}
buildForms(); fitRobot(); commit();
if (location.hash === '#firmware') showTab('firmware');
// builder.html?set=<path>: a simulator page sent the student here to set a value -> open its tab, highlight it
{
  const want = new URLSearchParams(location.search).get('set');
  const f = want && allFields(R).find((x) => x.path === want);
  if (f) {
    if (f.tab && f.tab !== 'top') showTab(f.tab);
    const target = f.tab === 'top' ? $(f.path === 'drive' ? '#drive' : '#robot-name')
      : ($(`[data-path="${CSS.escape(f.path)}"]`) || (f.sensor !== undefined ? $$('[data-panel="sensors"] .card')[f.sensor] : null));
    if (target) { target.scrollIntoView({ block: 'center' }); target.classList.add('flash'); if (target.focus) target.focus({ preventScroll: true }); }
    toast(t('builder.set_this', { name: f.key }), 'warn');
  }
}
// "Generate code for my robot" / "I have my own code" (builder.html?own=1 adds the three own-code steps)
{
  const own = isOwnMode(), head = $('.page-head') || $('h1');
  const bar = choiceBar(own);
  head.after(bar);
  if (own) { const box = el('section', { id: 'own' }); bar.after(box); mountOwn(box, () => R, showTab); }
}

// ---- 5. FIRMWARE: what hardware the robot has -> a ready-to-flash sketch ------------------------------------
function fwSel(options, value, onChange) {
  const s = el('select', {}, ...options.map(([v, label]) => el('option', { value: String(v), text: label })));
  s.value = String(value);
  s.addEventListener('change', () => onChange(s.value));
  return s;
}
function fwField(labelKey, control, noteKey) {
  const f = el('label', { class: 'field' }, el('span', { text: t(labelKey) }), control);
  return noteKey ? el('div', { class: 'stack' }, f, el('small', { class: 'muted', text: t(noteKey) })) : f;
}
// robot-file path of a value inside R.hardware (for "is it set")
function hwPath(obj, key) {
  const walk = (o, p) => { if (o === obj) return p; if (o && typeof o === 'object') for (const [k, v] of Object.entries(o)) { const r = walk(v, p + '.' + k); if (r) return r; } return null; };
  const p = walk(R.hardware, 'hardware'); return p ? `${p}.${key}` : null;
}
function markHw(obj, key) { const p = hwPath(obj, key); if (p) mark(p); }
// a board / driver / encoder choice fills in suggested pins: they count as set (SUGGESTED) until changed
function markSuggested() { for (const f of allFields(R)) if (f.path.startsWith('hardware.') && !isSet(R, f.path)) R.source[f.path] = 'SUGGESTED'; }
function pinSelect(obj, key, labelKey, vars) {
  const hw = R.hardware, B = BOARDS[hw.board];
  const opts = [[-1, '—'], ...B.pins.map((p) => [p, `${pinLabel(hw.board, p)}${pinTags(hw.board, p).length ? '  · ' + pinTags(hw.board, p).join(' ') : ''}`])];
  const s = fwSel(opts, obj[key] ?? -1, (v) => { obj[key] = +v; markHw(obj, key); fwChanged(false); });
  return el('label', { class: 'field' }, el('span', { text: t(labelKey, vars) }), s);
}
function fwNum(obj, key, labelKey, step = 'any') {
  const inp = el('input', { type: 'number', step, value: obj[key] });
  inp.addEventListener('input', () => { const v = parseFloat(inp.value); if (Number.isFinite(v)) { obj[key] = v; markHw(obj, key); fwChanged(false, true); } });
  return fwField(labelKey, inp);
}
// ticks/rev follows the encoder description until it has been measured
function syncTicks() {
  const c = computedTicksPerRev(R.hardware);
  if (c > 0 && !/MEASURED|IMPORTED/.test(R.source['wheels.ticks_per_rev'] || '')) { R.wheels.ticks_per_rev = c; R.source['wheels.ticks_per_rev'] = 'TYPED'; }
}
function fwChanged(rebuild, ticks = false) {
  if (ticks || rebuild) syncTicks();
  commit();
  if (rebuild) buildFirmware(); else fwTicksLine();
}
function signSel(obj, key, labelKey) { return fwField(labelKey, fwSel([[1, '+1'], [-1, '−1']], obj[key] < 0 ? -1 : 1, (v) => { obj[key] = +v; markHw(obj, key); fwChanged(false); })); }

function buildFirmware() {
  R.hardware = normalizeHardware(R.hardware, R.sensors.length);
  const hw = R.hardware, B = BOARDS[hw.board], box = $('[data-panel="firmware"]');
  box.innerHTML = '';
  box.append(el('p', { class: 'muted', text: t('fw.intro') }));
  // board
  const pinRows = B.pins.map((p) => el('tr', {}, el('td', { class: 'num', text: pinLabel(hw.board, p) }), el('td', { text: pinTags(hw.board, p).join(', ') || t('fw.plain_io') })));
  box.append(el('div', { class: 'card stack' }, el('h3', { text: t('fw.board_h') }),
    fwField('fw.board', fwSel(Object.entries(BOARDS).map(([k, b]) => [k, b.label]), hw.board, (v) => {
      mark('hardware.board'); hw.board = v; hw.i2c = { sda: BOARDS[v].i2c.sda, scl: BOARDS[v].i2c.scl };
      R.hardware = suggestPins(R, hw, true).hw; markSuggested(); fwChanged(true);
    }), 'fw.board_note.' + hw.board),
    el('details', {}, el('summary', { text: t('fw.pin_table') }), el('small', { class: 'muted', text: t('fw.pin_table_note.' + hw.board) }),
      el('table', {}, el('tr', {}, el('th', { text: t('fw.pin') }), el('th', { text: t('fw.can') })), ...pinRows))));
  if (hw.board === 'esp32') {
    const cb = el('input', { type: 'checkbox' }); cb.checked = hw.bt !== false;
    cb.addEventListener('change', () => { hw.bt = cb.checked; mark('hardware.bt'); fwChanged(true); });
    const nm = el('input', { value: hw.bt_name || '', placeholder: btName(R, { ...hw, bt_name: '' }), maxlength: 24 });
    nm.addEventListener('input', () => { hw.bt_name = nm.value; mark('hardware.bt_name'); fwChanged(false); });
    box.append(el('div', { class: 'card stack' }, el('h3', { text: t('fw.bt_h') }),
      el('label', { class: 'toggle' }, cb, el('span', { text: t('fw.bt_on') })),
      usesBt(hw) ? el('label', { class: 'field' }, el('span', { text: t('fw.bt_name') }), nm) : null,
      el('small', { class: 'muted', text: t('fw.bt_note') })));
  }
  // motor driver
  const d = hw.driver;
  const drv = el('div', { class: 'card stack' }, el('h3', { text: t('fw.driver_h') }),
    fwField('fw.driver', fwSel(DRIVERS.map((k) => [k, t('fw.drv.' + k)]), d.type, (v) => { d.type = v; mark('hardware.driver.type'); R.hardware = suggestPins(R, hw, true).hw; markSuggested(); fwChanged(true); }), 'fw.drv_note.' + d.type));
  for (const [side, m] of [['L', d.left], ['R', d.right]]) {
    const sideTxt = t(side === 'L' ? 'fw.left' : 'fw.right');
    if (d.type === 'pwm_dir') drv.append(pinSelect(m, 'a', 'fw.pin_pwm', { s: sideTxt }), pinSelect(m, 'b', 'fw.pin_dir', { s: sideTxt }));
    if (d.type === 'in1_in2') drv.append(pinSelect(m, 'a', 'fw.pin_in1', { s: sideTxt }), pinSelect(m, 'b', 'fw.pin_in2', { s: sideTxt }));
    if (d.type === 'en_in1_in2') drv.append(pinSelect(m, 'en', 'fw.pin_en', { s: sideTxt }), pinSelect(m, 'a', 'fw.pin_in1', { s: sideTxt }), pinSelect(m, 'b', 'fw.pin_in2', { s: sideTxt }));
  }
  if (d.type !== 'in1_in2') drv.append(pinSelect(d, 'stby', 'fw.pin_stby'));
  if (d.type !== 'pwm_dir') drv.append(fwField('fw.stop', fwSel([['brake', t('fw.stop_brake')], ['coast', t('fw.stop_coast')]], d.stop, (v) => { d.stop = v; mark('hardware.driver.stop'); fwChanged(false); }), d.type === 'in1_in2' ? 'fw.stop_note_in12' : 'fw.stop_note_en'));
  drv.append(el('div', { class: 'row' }, signSel(d, 'sign_l', 'fw.motor_sign_l'), signSel(d, 'sign_r', 'fw.motor_sign_r')), el('small', { class: 'muted', text: t('fw.sign_note') }));
  box.append(drv);
  // encoders
  const e = hw.encoder;
  const enc = el('div', { class: 'card stack' }, el('h3', { text: t('fw.enc_h') }),
    fwField('fw.enc', fwSel(ENCODERS.map((k) => [k, t('fw.enc.' + k)]), e.type, (v) => { e.type = v; mark('hardware.encoder.type'); R.hardware = suggestPins(R, hw, true).hw; markSuggested(); fwChanged(true, true); }), 'fw.enc_note.' + e.type));
  if (e.type === 'quadrature' && B.arch === 'avr') {
    enc.append(fwField('fw.avr_mode', fwSel([['x1', t('fw.avr_x1')], ['x4', t('fw.avr_x4')]], e.avr_mode, (v) => { e.avr_mode = v; mark('hardware.encoder.avr_mode'); R.hardware = suggestPins(R, hw, true).hw; markSuggested(); fwChanged(true, true); })));
  }
  if (e.type === 'quadrature') {
    for (const [side, c] of [['L', e.left], ['R', e.right]]) {
      const s = t(side === 'L' ? 'fw.left' : 'fw.right');
      enc.append(pinSelect(c, 'a', 'fw.pin_enc_a', { s }), pinSelect(c, 'b', 'fw.pin_enc_b', { s }));
    }
  }
  if (e.type === 'single') enc.append(pinSelect(e.left, 'a', 'fw.pin_enc', { s: t('fw.left') }), pinSelect(e.right, 'a', 'fw.pin_enc', { s: t('fw.right') }));
  if (e.type === 'as5600') {
    enc.append(fwField('fw.as5600_bus', fwSel([['mux', t('fw.as5600_mux')], ...(B.arch === 'esp32' ? [['two_buses', t('fw.as5600_two')]] : [])], e.as5600_bus,
      (v) => { e.as5600_bus = v; mark('hardware.encoder.as5600_bus'); R.hardware = suggestPins(R, hw, true).hw; markSuggested(); fwChanged(true); })));
    if (e.as5600_bus === 'mux') enc.append(el('div', { class: 'row' }, fwNum(e, 'mux_ch_l', 'fw.mux_ch_l', 1), fwNum(e, 'mux_ch_r', 'fw.mux_ch_r', 1)));
    else enc.append(pinSelect(e, 'sda2', 'fw.pin_sda2'), pinSelect(e, 'scl2', 'fw.pin_scl2'));
  }
  if (e.type === 'quadrature' || e.type === 'single') enc.append(fwNum(e, 'ppr', e.type === 'single' ? 'fw.slots' : 'fw.ppr'));
  if (e.type !== 'none') enc.append(fwNum(e, 'gear', e.type === 'as5600' ? 'fw.gear_magnet' : 'fw.gear'));
  if (e.type === 'quadrature' || e.type === 'as5600') enc.append(el('div', { class: 'row' }, signSel(e, 'sign_l', 'fw.enc_sign_l'), signSel(e, 'sign_r', 'fw.enc_sign_r')));
  enc.append(el('p', { id: 'fw-ticks', class: 'muted' }));
  box.append(enc);
  // I2C + sensors
  const sens = el('div', { class: 'card stack' }, el('h3', { text: t('fw.sensors_h') }), el('p', { class: 'muted', text: t('fw.sensors_note') }));
  if (B.arch === 'esp32') sens.append(el('div', { class: 'row' }, pinSelect(hw.i2c, 'sda', 'fw.pin_sda'), pinSelect(hw.i2c, 'scl', 'fw.pin_scl')));
  else sens.append(el('p', { class: 'muted', text: t('fw.i2c_fixed', { sda: pinLabel(hw.board, B.i2c.sda), scl: pinLabel(hw.board, B.i2c.scl) }) }));
  R.sensors.forEach((s, i) => {
    const p = hw.sensors[i], row = el('div', { class: 'stack', style: `border-inline-start:4px solid ${PALETTE[i % PALETTE.length]};padding-inline-start:8px` },
      el('b', { text: `#${i + 1} ${sensorWireName(s, i)} — ${t('sensor.' + s.type)} (${s.angle_deg}°)` }));
    if (/^VL53/.test(s.type)) {
      const addr = el('input', { value: '0x' + (+p.addr).toString(16), style: 'width:70px' });
      addr.addEventListener('change', () => { const v = parseInt(addr.value, 16); if (v >= 0x08 && v < 0x78) { p.addr = v; fwChanged(false); } });
      row.append(el('div', { class: 'row' }, pinSelect(p, 'xshut', 'fw.pin_xshut'), fwField('fw.addr', addr)));
    } else if (s.type === 'HC_SR04') row.append(el('div', { class: 'row' }, pinSelect(p, 'trig', 'fw.pin_trig'), pinSelect(p, 'echo', 'fw.pin_echo')));
    else row.append(pinSelect(p, 'analog', 'fw.pin_analog'));
    sens.append(row);
  });
  box.append(sens);
  // checks + download
  box.append(el('div', { class: 'row' }, el('button', { text: t('fw.suggest'), onclick: () => {
    const { hw: h, failed } = suggestPins(R, R.hardware, true); R.hardware = h; fwChanged(true);
    if (failed.length) toast(t('fw.suggest_failed', { roles: failed.join(', ') }), 'bad');
  } }), el('small', { class: 'muted', text: t('fw.suggest_note') })));
  box.append(el('div', { id: 'fw-checks', class: 'card' }));
  const libs = el('ul', { id: 'fw-libs' });
  const pre = el('pre', { id: 'fw-code' });
  const det = el('details', {}, el('summary', { text: t('fw.preview') }), pre);
  det.addEventListener('toggle', () => { if (det.open) pre.textContent = generateSketch(R, R.hardware).code; });
  box.append(el('div', { class: 'card stack' }, el('h3', { text: t('fw.download_h') }),
    el('div', { class: 'row' }, el('button', { class: 'primary', id: 'fw-dl', text: t('fw.download'), onclick: () => {
      const g = generateSketch(R, R.hardware);
      if (hasErrors(g.messages) && !confirm(t('fw.confirm_errors'))) return;
      download(g.fileName, g.code, 'text/plain');
    } })),
    el('p', { class: 'muted', text: t('fw.download_note') }),
    el('b', { text: t('fw.libs_h') }), libs, el('p', { class: 'muted', id: 'fw-core' }), det));
  fwTicksLine(); fwChecks();
}
function fwTicksLine() {
  const p = $('#fw-ticks'); if (!p) return;
  const hw = R.hardware, c = computedTicksPerRev(hw), used = sketchTicks(R, hw);
  if (hw.encoder.type === 'none') { p.textContent = t('fw.ticks_none'); return; }
  p.textContent = t('fw.ticks_line', { c: +c.toFixed(1), e: edgesPerPulse(hw), u: +(+used).toFixed(1) }) + (isLowRes(hw, used) ? ' ' + t('fw.ticks_low') : '');
}
function fwChecks() {
  const box = $('#fw-checks'); if (!box) return;
  const list = validateHardware(R, R.hardware);
  const order = { bad: 0, warn: 1, info: 2 };
  list.sort((a, b) => order[a.level] - order[b.level]);
  box.innerHTML = '';
  box.append(el('h3', { text: t('fw.checks_h') }));
  if (!hasErrors(list)) box.append(el('p', {}, el('span', { class: 'pill ok', text: t('chk.ok') }), ' ', t('fw.pins_ok')));
  for (const m of list) box.append(el('p', {}, el('span', { class: 'pill ' + (m.level === 'info' ? '' : m.level), text: m.level === 'info' ? t('fw.info') : t('chk.' + m.level) }), ' ', t(m.key, m.vars)));
  const libs = $('#fw-libs');
  if (libs) {
    const L = requiredLibraries(R, R.hardware);
    libs.innerHTML = '';
    if (!L.length) libs.append(el('li', { text: t('fw.libs_none') }));
    for (const l of L) libs.append(el('li', {}, el('code', { text: l.name }), ` (${l.by}) — ${l.why}`));
    $('#fw-core').textContent = t(BOARDS[R.hardware.board].arch === 'esp32' ? 'fw.core_esp32' : 'fw.core_avr');
  }
  const pre = $('#fw-code');
  if (pre && pre.closest('details').open) pre.textContent = generateSketch(R, R.hardware).code;
}
