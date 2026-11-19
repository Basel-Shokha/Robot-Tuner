// builder.js - 1. ROBOT BUILDER: draw + edit the robot, sensor library, JSON / robot_hardware.txt.
import { page, $, $$, el, t, download, pickFile, toast } from './common.js';
import { loadRobot, saveRobot, defaultRobot, makeSensor, migrate, SENSOR_LIBRARY, SENSOR_FIELDS, robotToHardwareTxt, hardwareTxtToRobot, downloadName } from './store.js';
import { View, robotShapes, css, PALETTE } from './draw.js';

await page();
let R = loadRobot();
let selected = -1;  // selected sensor
const view = new View($('#cv'), { scale: 1800, aspect: 0.75, minH: 360, maxH: 680 });

function commit(redrawForm = false) {
  saveRobot(R);
  if (redrawForm) buildForms();
  draw(); checks();
  $('#json').textContent = JSON.stringify(R, null, 1);
}
function mark(path, tag = 'TYPED') { R.source[path] = tag; }

// ---- numeric field bound to R[group][key] -------------------------------------------------------
function num(group, key, labelKey, unit, opts = {}) {
  const obj = typeof group === 'string' ? R[group] : group;
  const path = opts.path || `${group}.${key}`;
  const inp = el('input', { type: 'number', step: opts.step || 'any', value: obj[key], min: opts.min });
  inp.addEventListener('input', () => {
    const v = parseFloat(inp.value);
    if (!Number.isFinite(v)) return;
    obj[key] = v; mark(path); src.textContent = 'TYPED'; src.className = 'pill src'; commit();
  });
  const tag = R.source[path] || 'DEFAULT';
  const src = el('span', { class: 'pill src ' + (tag === 'MEASURED' ? 'ok' : ''), text: tag, title: t('builder.source_hint') });
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
  cb.addEventListener('change', () => { R.caster.enabled = cb.checked; commit(); });
  cas.append(el('label', { class: 'toggle' }, cb, el('span', { text: t('builder.caster_on') })), num('caster', 'x_mm', 'builder.caster_x', 'mm'), num('caster', 'diameter_mm', 'builder.caster_d', 'mm'));
  if (R.drive === '2wd') g.append(cas);
  // sensors
  const s = $('[data-panel="sensors"]'); s.innerHTML = '';
  s.append(el('p', { class: 'muted', text: t('builder.sensors_intro') }));
  R.sensors.forEach((sen, i) => s.append(sensorCard(sen, i)));
  const add = el('button', { text: t('builder.add_sensor'), disabled: R.sensors.length >= 5 ? true : null, onclick: () => {
    R.sensors.push(makeSensor(R.sensors[0] ? R.sensors[0].type : 'VL53L1X_short', { x_mm: R.body.front_mm - 10, angle_deg: 0, name: 's' + R.sensors.length }));
    selected = R.sensors.length - 1; commit(true);
  } });
  s.append(el('div', { class: 'row' }, add, el('span', { class: 'muted', text: t('builder.max5') })));
  // motor + timing
  const m = $('[data-panel="motor"]'); m.innerHTML = '';
  m.append(el('div', { class: 'card stack' }, el('h3', { text: t('builder.drive_h') }),
    num('wheels', 'ticks_per_rev', 'builder.ticks', t('unit.ticks')), num('motor', 'max_speed_mps', 'builder.max_speed', 'm/s'),
    num('motor', 'step_tau_s', 'builder.step_tau', 's'), el('small', { class: 'muted', text: t('builder.step_tau_note') })));
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
}

function sensorCard(sen, i) {
  const card = el('div', { class: 'card stack' + (i === selected ? ' selected' : ''), style: `border-inline-start:4px solid ${PALETTE[i % PALETTE.length]}` });
  const type = el('select', {}, ...Object.keys(SENSOR_LIBRARY).map((k) => el('option', { value: k, text: t('sensor.' + k) })));
  type.value = sen.type;
  type.addEventListener('change', () => {
    const fresh = makeSensor(type.value, sen);
    Object.assign(sen, fresh, { x_mm: sen.x_mm, y_mm: sen.y_mm, angle_deg: sen.angle_deg, name: sen.name });
    commit(true);
  });
  const name = el('input', { value: sen.name || '', style: 'width:90px' });
  name.addEventListener('input', () => { sen.name = name.value; commit(); });
  const rm = el('button', { class: 'small', text: '✕', title: t('builder.remove'), onclick: () => { R.sensors.splice(i, 1); selected = -1; commit(true); } });
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
  if (S.caster) { view.dot(S.caster.x / 1000, 0, Math.max(3, S.caster.d / 2000 * view.scale), '#888'); }
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
  R.sensors.forEach((s, i) => { const d = Math.hypot(s.x_mm / 1000 - x, s.y_mm / 1000 - y); if (d < bd) { bd = d; best = i; } });
  if (best < 0) return false;
  selected = best; drag = { i: best, rotate: e.shiftKey }; view.c.setPointerCapture(e.pointerId); return true;
};
view.onPointerMove = (e) => {
  if (!drag) return false;
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
}));
$('#robot-name').addEventListener('input', (e) => { R.name = e.target.value; commit(); });
$('#drive').addEventListener('change', (e) => { R.drive = e.target.value; if (R.drive === '4wd') R.caster.enabled = false; commit(true); fitRobot(); draw(); });
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
  r.caster = { enabled: drive === '2wd', x_mm: 90, diameter_mm: 15 };
  r.motor = { ...r.motor, model: 'lag', max_speed_mps: 0.5, step_tau_s: 0.12 };
  r.sensors = [makeSensor('VL53L0X', { name: 'right', x_mm: 110, y_mm: -40, angle_deg: -45 }), makeSensor('VL53L0X', { name: 'front', x_mm: 120, y_mm: 0, angle_deg: 0 }),
    makeSensor('VL53L0X', { name: 'left', x_mm: 110, y_mm: 40, angle_deg: 45 })];
  r.timing = { loop_ms: 33, age_ms: 25, age_jitter_ms: 15 };
  r.pid = { ...r.pid, kp: 1.0, ki: 0.0, kd: 0.1, speed_mps: 0.3 };
  return r;
}
$('#new-2wd').addEventListener('click', () => { if (confirm(t('builder.confirm_new'))) { R = blank('2wd'); commit(true); fitRobot(); draw(); } });
$('#new-4wd').addEventListener('click', () => { if (confirm(t('builder.confirm_new'))) { R = blank('4wd'); commit(true); fitRobot(); draw(); } });
$('#reset-example').addEventListener('click', () => { if (confirm(t('builder.confirm_new'))) { R = defaultRobot(); commit(true); fitRobot(); draw(); } });

buildForms(); fitRobot(); commit();
