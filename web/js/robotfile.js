// robotfile.js - the complete robot file: EVERY value that describes the robot's hardware (body, wheels, caster,
// motors, encoders, sensors, timing, board and pins) plus the controller settings, one "key value" per line.
// A value the student has not set is written as N/A. Importing N/A leaves that value unset.
//
// "Set" = the robot's `source` has a tag for it (TYPED, MEASURED, IMPORTED, DATASHEET ...). Unset values still hold a
// placeholder number inside the robot (drawing, sim fallback), but they are N/A in the file, and the simulator pages
// refuse to start until the REQUIRED ones are set (missingForSim).
import { defaultRobot, migrate, makeSensor, SENSOR_LIBRARY, SENSOR_FIELDS, isSet } from './store.js';
import { BOARDS, normalizeHardware, ENCODERS, DRIVERS } from './firmware-gen.js';

export const FILE_HEADER = '# robot-tuner robot file v1';
const NUM = 'num', WORD = 'word';

// ---- paths into the robot ----
export function getPath(r, path) { return path.split('.').reduce((o, k) => (o == null ? undefined : o[k]), r); }
export function setPath(r, path, v) {
  const ks = path.split('.'), last = ks.pop();
  const o = ks.reduce((x, k) => (x[k] == null ? (x[k] = {}) : x[k]), r);
  o[last] = v;
}

// ---- the field list. tab = the Robot builder tab where the student sets it; req(r) = needed to simulate ----
const encOn = (r) => (r.hardware || {}).encoder?.type !== 'none';
const drv = (r) => (r.hardware || {}).driver?.type;
const enc = (r) => (r.hardware || {}).encoder?.type;
const avr = (r) => BOARDS[(r.hardware || {}).board || 'esp32']?.arch === 'avr';
const always = () => true;
function robotFields() {
  return [
    // section, key, path, type, unit, tab, req, when, note
    ['General', 'name', 'name', WORD, '', 'top', null, always, 'robot name'],
    ['General', 'drive', 'drive', WORD, '2wd | 4wd', 'top', always, always, '2 driven wheels + caster, or 4-wheel skid steer'],
    ['Body (mm, from the middle of the wheel axle)', 'body_front_mm', 'body.front_mm', NUM, 'mm', 'geom', always, always, 'front edge ahead of the axle'],
    ['Body (mm, from the middle of the wheel axle)', 'body_rear_mm', 'body.rear_mm', NUM, 'mm', 'geom', always, always, 'rear edge behind the axle'],
    ['Body (mm, from the middle of the wheel axle)', 'body_width_mm', 'body.width_mm', NUM, 'mm', 'geom', always, always, ''],
    ['Wheels', 'wheel_diameter_mm', 'wheels.diameter_mm', NUM, 'mm', 'geom', always, always, 'with the tyre'],
    ['Wheels', 'wheel_width_mm', 'wheels.width_mm', NUM, 'mm', 'geom', null, always, ''],
    ['Wheels', 'wheel_track_mm', 'wheels.track_mm', NUM, 'mm', 'geom', always, always, 'wheel centre to centre'],
    ['Wheels', 'wheel_x_mm', 'wheels.x_mm', NUM, 'mm', 'geom', null, always, 'axle position (0 = origin)'],
    ['Wheels', 'wheelbase_mm', 'wheels.wheelbase_mm', NUM, 'mm', 'geom', (r) => r.drive === '4wd', (r) => r.drive === '4wd', 'front to rear axle (4wd)'],
    ['Wheels', 'track_w_effective_mm', 'wheels.track_effective_mm', NUM, 'mm', 'geom', null, always, 'from CAL SPIN (Measure)'],
    ['Wheels', 'enc_ticks_per_rev', 'wheels.ticks_per_rev', NUM, 'ticks per wheel turn', 'motor', encOn, encOn, 'from CAL TICKS (Measure)'],
    ['Wheels', 'wheel_scale_left', 'wheels.scale_l', NUM, 'factor', 'geom', null, encOn, 'from CAL DRIVE (Measure)'],
    ['Wheels', 'wheel_scale_right', 'wheels.scale_r', NUM, 'factor', 'geom', null, encOn, 'from CAL DRIVE (Measure)'],
    ['Caster ball', 'caster', 'caster.enabled', WORD, 'yes | no', 'geom', null, (r) => r.drive === '2wd', ''],
    ['Caster ball', 'caster_x_mm', 'caster.x_mm', NUM, 'mm', 'geom', null, (r) => r.drive === '2wd' && r.caster.enabled, '+ = ahead of the axle'],
    ['Caster ball', 'caster_y_mm', 'caster.y_mm', NUM, 'mm', 'geom', null, (r) => r.drive === '2wd' && r.caster.enabled, '+ = left of centre'],
    ['Caster ball', 'caster_diameter_mm', 'caster.diameter_mm', NUM, 'mm', 'geom', null, (r) => r.drive === '2wd' && r.caster.enabled, ''],
    ['Motors', 'max_speed_mps', 'motor.max_speed_mps', NUM, 'm/s', 'motor', always, always, 'top wheel speed, from CAL MAXSPEED (Measure)'],
    ['Motors', 'v_min_mps', 'motor.v_min_mps', NUM, 'm/s', 'motor', null, always, 'slowest steady speed, from CAL VMIN (Measure)'],
    ['Motors', 'step_tau_s', 'motor.step_tau_s', NUM, 's', 'motor', always, always, 'time to reach 63 % of a new speed, from CAL STEP (Measure)'],
    ['Motors', 'motor_model', 'motor.model', WORD, 'pid | lag', 'motor', null, always, 'pid = wheel PID on a DC-motor model (needs the motor_* values below)'],
    ['Motors', 'pwm_max', 'motor.pwm_max', NUM, 'PWM', 'motor', null, always, ''],
    ['Motors', 'pwm_limit', 'motor.pwm_limit', NUM, 'PWM', 'motor', null, always, ''],
    ['Motors', 'battery_v', 'motor.battery_v', NUM, 'V', 'motor', null, always, ''],
    ['Motors', 'motor_nominal_v', 'motor.nominal_v', NUM, 'V', 'motor', null, always, 'battery voltage during the motor measurements'],
    ['Motors', 'motor_k_left', 'motor.k_l', NUM, '(m/s) per PWM', 'motor', null, always, ''],
    ['Motors', 'motor_k_right', 'motor.k_r', NUM, '(m/s) per PWM', 'motor', null, always, ''],
    ['Motors', 'motor_static_left', 'motor.static_l', NUM, 'PWM', 'motor', null, always, ''],
    ['Motors', 'motor_static_right', 'motor.static_r', NUM, 'PWM', 'motor', null, always, ''],
    ['Motors', 'motor_breakaway_left', 'motor.breakaway_l', NUM, 'PWM', 'motor', null, always, 'PWM to start from rest'],
    ['Motors', 'motor_breakaway_right', 'motor.breakaway_r', NUM, 'PWM', 'motor', null, always, ''],
    ['Motors', 'motor_tau_left_s', 'motor.tau_l_s', NUM, 's', 'motor', null, always, ''],
    ['Motors', 'motor_tau_right_s', 'motor.tau_r_s', NUM, 's', 'motor', null, always, ''],
    ['Wheel speed loop', 'wheel_pid_period_ms', 'motor.wheel_pid_period_ms', NUM, 'ms', 'motor', null, always, ''],
    ['Wheel speed loop', 'wheel_kp', 'motor.wheel_kp', NUM, 'PWM per (tick per period)', 'motor', null, always, ''],
    ['Wheel speed loop', 'wheel_ki', 'motor.wheel_ki', NUM, 'PWM per (tick per period), summed once per period', 'motor', null, always, ''],
    ['Wheel speed loop', 'wheel_kd', 'motor.wheel_kd', NUM, 'PWM per (tick per period) change per period', 'motor', null, always, ''],
    ['Wheel speed loop', 'wheel_ff_static', 'motor.ff_static', NUM, 'PWM', 'motor', null, always, 'from CAL FF (Measure)'],
    ['Wheel speed loop', 'wheel_ff_gain', 'motor.ff_gain', NUM, 'PWM per (tick per period)', 'motor', null, always, 'from CAL FF (Measure)'],
    ['Wheel speed loop', 'speed_smoothing_ms', 'motor.speed_tau_ms', NUM, 'ms', 'motor', null, always, '0 = off (Robot Tuner firmware)'],
    ['Timing', 'control_period_ms', 'timing.loop_ms', NUM, 'ms', 'motor', always, always, 'one steering step per fresh set of sensor readings (STATUS)'],
    ['Timing', 'reading_age_ms', 'timing.age_ms', NUM, 'ms', 'motor', null, always, ''],
    ['Timing', 'reading_age_jitter_ms', 'timing.age_jitter_ms', NUM, 'ms', 'motor', null, always, ''],
    ['Steering PID', 'pid_kp', 'pid.kp', NUM, '(m/s) per m', null, null, always, 'from Train PID'],
    ['Steering PID', 'pid_ki', 'pid.ki', NUM, '(m/s) per (m*s)', null, null, always, ''],
    ['Steering PID', 'pid_kd', 'pid.kd', NUM, '(m/s) per (m/s)', null, null, always, ''],
    ['Steering PID', 'pid_speed_frac', 'pid.speed_frac', NUM, 'fraction 0..1', null, null, always, 'cruise = v_min + speed_frac (v_max - v_min)'],
    ['Steering PID', 'pid_error_clip_mm', 'pid.error_clip_mm', NUM, 'mm', null, null, always, ''],
    ['Steering PID', 'pid_integral_limit', 'pid.integral_limit', NUM, 'm*s', null, null, always, ''],
    ['Steering PID', 'pid_d_filter_alpha', 'pid.d_filter_alpha', NUM, 'per 50 ms', null, null, always, ''],
    ['Board and pins', 'board', 'hardware.board', WORD, Object.keys(BOARDS).join(' | '), 'firmware', null, always, ''],
    ['Board and pins', 'motor_driver', 'hardware.driver.type', WORD, DRIVERS.join(' | '), 'firmware', null, always, ''],
    ['Board and pins', 'motor_stop', 'hardware.driver.stop', WORD, 'brake | coast', 'firmware', null, (r) => drv(r) !== 'pwm_dir', ''],
    ['Board and pins', 'pin_motor_left_en', 'hardware.driver.left.en', NUM, 'pin', 'firmware', null, (r) => drv(r) === 'en_in1_in2', ''],
    ['Board and pins', 'pin_motor_left_a', 'hardware.driver.left.a', NUM, 'pin', 'firmware', null, always, 'IN1 or PWM'],
    ['Board and pins', 'pin_motor_left_b', 'hardware.driver.left.b', NUM, 'pin', 'firmware', null, always, 'IN2 or DIR'],
    ['Board and pins', 'pin_motor_right_en', 'hardware.driver.right.en', NUM, 'pin', 'firmware', null, (r) => drv(r) === 'en_in1_in2', ''],
    ['Board and pins', 'pin_motor_right_a', 'hardware.driver.right.a', NUM, 'pin', 'firmware', null, always, 'IN1 or PWM'],
    ['Board and pins', 'pin_motor_right_b', 'hardware.driver.right.b', NUM, 'pin', 'firmware', null, always, 'IN2 or DIR'],
    ['Board and pins', 'pin_motor_stby', 'hardware.driver.stby', NUM, 'pin (-1 = none)', 'firmware', null, (r) => drv(r) !== 'in1_in2', ''],
    ['Board and pins', 'motor_left_sign', 'hardware.driver.sign_l', NUM, '+1 | -1', 'firmware', null, always, 'check with CAL DIR'],
    ['Board and pins', 'motor_right_sign', 'hardware.driver.sign_r', NUM, '+1 | -1', 'firmware', null, always, ''],
    ['Board and pins', 'encoder_type', 'hardware.encoder.type', WORD, ENCODERS.join(' | '), 'firmware', null, always, ''],
    ['Board and pins', 'enc_decoding', 'hardware.encoder.avr_mode', WORD, 'x1 | x4', 'firmware', null, (r) => avr(r) && enc(r) === 'quadrature', 'Uno / Mega quadrature'],
    ['Board and pins', 'enc_ppr', 'hardware.encoder.ppr', NUM, 'pulses per motor turn', 'firmware', null, (r) => ['quadrature', 'single'].includes(enc(r)), ''],
    ['Board and pins', 'enc_gear', 'hardware.encoder.gear', NUM, 'motor turns per wheel turn', 'firmware', null, (r) => enc(r) !== 'none', ''],
    ['Board and pins', 'pin_enc_left_a', 'hardware.encoder.left.a', NUM, 'pin', 'firmware', null, (r) => ['quadrature', 'single'].includes(enc(r)), ''],
    ['Board and pins', 'pin_enc_left_b', 'hardware.encoder.left.b', NUM, 'pin', 'firmware', null, (r) => enc(r) === 'quadrature', ''],
    ['Board and pins', 'pin_enc_right_a', 'hardware.encoder.right.a', NUM, 'pin', 'firmware', null, (r) => ['quadrature', 'single'].includes(enc(r)), ''],
    ['Board and pins', 'pin_enc_right_b', 'hardware.encoder.right.b', NUM, 'pin', 'firmware', null, (r) => enc(r) === 'quadrature', ''],
    ['Board and pins', 'enc_left_sign', 'hardware.encoder.sign_l', NUM, '+1 | -1', 'firmware', null, (r) => ['quadrature', 'as5600'].includes(enc(r)), 'check with CAL DIR'],
    ['Board and pins', 'enc_right_sign', 'hardware.encoder.sign_r', NUM, '+1 | -1', 'firmware', null, (r) => ['quadrature', 'as5600'].includes(enc(r)), ''],
    ['Board and pins', 'as5600_bus', 'hardware.encoder.as5600_bus', WORD, 'mux | two_buses', 'firmware', null, (r) => enc(r) === 'as5600', ''],
    ['Board and pins', 'as5600_mux_addr', 'hardware.encoder.mux_addr', NUM, 'I2C address', 'firmware', null, (r) => enc(r) === 'as5600', ''],
    ['Board and pins', 'as5600_mux_ch_left', 'hardware.encoder.mux_ch_l', NUM, 'channel', 'firmware', null, (r) => enc(r) === 'as5600', ''],
    ['Board and pins', 'as5600_mux_ch_right', 'hardware.encoder.mux_ch_r', NUM, 'channel', 'firmware', null, (r) => enc(r) === 'as5600', ''],
    ['Board and pins', 'pin_i2c2_sda', 'hardware.encoder.sda2', NUM, 'pin', 'firmware', null, (r) => enc(r) === 'as5600', ''],
    ['Board and pins', 'pin_i2c2_scl', 'hardware.encoder.scl2', NUM, 'pin', 'firmware', null, (r) => enc(r) === 'as5600', ''],
    ['Board and pins', 'pin_i2c_sda', 'hardware.i2c.sda', NUM, 'pin', 'firmware', null, (r) => !avr(r), ''],
    ['Board and pins', 'pin_i2c_scl', 'hardware.i2c.scl', NUM, 'pin', 'firmware', null, (r) => !avr(r), ''],
  ];
}
const SENSOR_POSE = [['type', WORD, Object.keys(SENSOR_LIBRARY).join(' | '), true], ['name', WORD, '', false],
  ['x_mm', NUM, 'mm', true], ['y_mm', NUM, 'mm (+ = left)', true], ['angle_deg', NUM, 'deg (+ = looking left)', true]];
const SENSOR_UNITS = { fov_deg: 'deg', min_mm: 'mm', max_mm: 'mm', period_ms: 'ms', noise_mm: 'mm', noise_pct: '%', dropout_pct: '%', rays: '', max_incidence_deg: 'deg', bias_mm: 'mm' };
const SENSOR_PINS = [['xshut', 'pin', (t) => t.startsWith('VL53')], ['addr', 'I2C address', (t) => t.startsWith('VL53')], ['trig', 'pin', (t) => t === 'HC_SR04'],
  ['echo', 'pin', (t) => t === 'HC_SR04'], ['analog', 'pin', (t) => t.startsWith('GP2')]];

/** every field of this robot: {section, key, path, type, unit, tab, required, note} */
export function allFields(r) {
  const out = [];
  for (const [section, key, path, type, unit, tab, req, when, note] of robotFields()) {
    if (!when(r)) continue;
    out.push({ section, key, path, type, unit, tab, required: !!(req && req(r)), note });
  }
  out.push({ section: 'Sensors', key: `sensor_count`, path: 'sensors.length', type: NUM, unit: '1..5', tab: 'sensors', required: true, note: '' });
  r.sensors.forEach((s, i) => {
    const n = i + 1, sec = `Sensor ${n}`;
    for (const [f, type, unit, req] of SENSOR_POSE) out.push({ section: sec, key: `sensor${n}_${f}`, path: `sensors.${i}.${f}`, type, unit, tab: 'sensors', required: req, note: '', sensor: i });
    for (const f of SENSOR_FIELDS) out.push({ section: sec, key: `sensor${n}_${f}`, path: `sensors.${i}.${f}`, type: NUM, unit: SENSOR_UNITS[f], tab: 'sensors', required: false, note: f === 'bias_mm' ? 'reading - true, from CAL TOF (Measure)' : '', sensor: i });
    for (const [f, unit, when] of SENSOR_PINS) if (when(s.type)) out.push({ section: sec, key: `sensor${n}_${f}`, path: `hardware.sensors.${i}.${f}`, type: NUM, unit, tab: 'firmware', required: false, note: '', sensor: i });
  });
  return out;
}

/** the required fields that are not set yet (the simulator pages send the student to the first one) */
export function missingForSim(r) { return allFields(r).filter((f) => f.required && f.path !== 'sensors.length' && !isSet(r, f.path)); }

/** builder link that opens the right tab and highlights the field */
export function fieldLink(f) { return `builder.html?set=${encodeURIComponent(f.path)}`; }

// ---- write / read ----
const fmt = (v) => (typeof v === 'number' ? String(+v.toPrecision(8)) : typeof v === 'boolean' ? (v ? 'yes' : 'no') : String(v).replace(/\s+/g, '_'));
export function robotToFile(r0) {
  const r = migrate(JSON.parse(JSON.stringify(r0)));
  r.hardware = normalizeHardware(r.hardware, r.sensors.length);
  const L = [FILE_HEADER,
    '# Written by Robot Tuner. Everything about the robot\'s hardware, one "key value" per line, then # status, unit, note.',
    '# N/A = not set yet. Status: TYPED, MEASURED, IMPORTED, DATASHEET (from the sensor type) ... * = needed before simulating.',
    '# Frame: origin = middle of the drive axle, +x forward, +y LEFT, angles counter-clockwise (deg).',
    '# Load it again in Robot builder -> File -> Load robot file.'];
  let sec = '';
  for (const f of allFields(r)) {
    if (f.section !== sec) { sec = f.section; L.push('', `# ---- ${sec} ----`); }
    const set = f.path === 'sensors.length' || isSet(r, f.path);
    const v = f.path === 'sensors.length' ? r.sensors.length : getPath(r, f.path);
    const val = set && v !== undefined && v !== null && v !== '' ? fmt(v) : 'N/A';
    const tag = f.path === 'sensors.length' ? 'TYPED' : (set ? sourceTag(r, f.path) : 'NOT SET');
    L.push(`${f.key.padEnd(26)} ${val.padStart(12)}   # ${f.required ? '*' : ' '}${tag}${f.unit ? '  ' + f.unit : ''}${f.note ? '  - ' + f.note : ''}`);
  }
  return L.join('\n') + '\n';
}
function sourceTag(r, path) {
  const s = r.source || {};
  const m = /^sensors\.(\d+)\.(\w+)$/.exec(path);
  if (m) return s[path] || (['x_mm', 'y_mm', 'angle_deg', 'name'].includes(m[2]) ? s[`sensors.${m[1]}`] : s[`sensors.${m[1]}.type`] ? 'DATASHEET' : '') || 'TYPED';
  return s[path] || 'TYPED';
}
export const isRobotFile = (text) => text.trimStart().startsWith(FILE_HEADER);

/** read a robot file; N/A values stay unset. Unknown keys are reported, not fatal. */
export function fileToRobot(text) {
  const kv = {};
  for (const raw of text.split('\n')) {
    const line = raw.replace(/#.*/, '').trim();
    if (!line) continue;
    const [k, v] = line.split(/\s+/);
    kv[k] = v;
  }
  const r = defaultRobot();
  r.source = {};
  const n = Math.max(1, Math.min(5, parseInt(kv.sensor_count, 10) || 3));
  r.sensors = Array.from({ length: n }, (_, i) => r.sensors[i] || makeSensor('VL53L1X_short', { name: 's' + i }));
  r.hardware = normalizeHardware(r.hardware, n);
  for (let i = 0; i < n; i++) { const ty = kv[`sensor${i + 1}_type`]; if (ty && SENSOR_LIBRARY[ty]) r.sensors[i] = { ...makeSensor(ty), name: r.sensors[i].name }; }
  if (kv.drive === '2wd' || kv.drive === '4wd') r.drive = kv.drive;
  if (kv.caster === 'yes' || kv.caster === 'no') r.caster.enabled = kv.caster === 'yes';
  if (kv.encoder_type) r.hardware.encoder.type = kv.encoder_type;
  if (kv.motor_driver) r.hardware.driver.type = kv.motor_driver;
  if (kv.board && BOARDS[kv.board]) r.hardware.board = kv.board;
  const unknown = [];
  const fields = allFields(r), known = new Set(fields.map((f) => f.key));
  for (const k of Object.keys(kv)) if (!known.has(k)) unknown.push(k);
  for (const f of fields) {
    const v = kv[f.key];
    if (v === undefined || v === 'N/A' || f.path === 'sensors.length') continue;
    let x;
    if (f.type === NUM) { x = /^0x/i.test(v) ? parseInt(v, 16) : parseFloat(v); if (!Number.isFinite(x)) { unknown.push(f.key); continue; } }
    else x = f.path === 'caster.enabled' ? v === 'yes' : (f.path === 'name' ? v.replace(/_/g, ' ') : v);
    setPath(r, f.path, x);
    r.source[srcKey(f.path)] = 'IMPORTED';
  }
  return { robot: migrate(r), unknown };
}
// builder marks the sensor pose as "sensors.i" (one tag for x, y and angle)
function srcKey(path) { const m = /^sensors\.(\d+)\.(x_mm|y_mm|angle_deg)$/.exec(path); return m ? `sensors.${m[1]}` : path; }

/** our own robot, every value set (Simulation-To-Real-Life robot_hardware.txt): for tests and the example file */
export function exampleRobot() {
  const r = defaultRobot();
  r.name = 'Basel & Nicole robot (ESP32, DRV8833, 3x VL53L1X)';
  const measured = ['body.front_mm', 'body.rear_mm', 'body.width_mm', 'wheels.diameter_mm', 'wheels.width_mm', 'wheels.track_mm', 'wheels.x_mm',
    'wheels.track_effective_mm', 'wheels.ticks_per_rev', 'wheels.scale_l', 'wheels.scale_r', 'caster.x_mm', 'caster.diameter_mm', 'motor.max_speed_mps',
    'motor.v_min_mps', 'motor.step_tau_s', 'motor.k_l', 'motor.k_r', 'motor.static_l', 'motor.static_r', 'motor.breakaway_l', 'motor.breakaway_r',
    'motor.tau_l_s', 'motor.tau_r_s', 'motor.ff_gain', 'timing.loop_ms', 'timing.age_ms', 'sensors.0', 'sensors.1', 'sensors.2', 'sensors.1.bias_mm'];
  r.source = {};
  for (const f of allFields(r)) if (f.path !== 'sensors.length') r.source[srcKey(f.path)] = 'TYPED';
  for (const p of measured) r.source[p] = 'MEASURED';
  for (let i = 0; i < r.sensors.length; i++) for (const f of SENSOR_FIELDS) if (f !== 'bias_mm') delete r.source[`sensors.${i}.${f}`];   // = DATASHEET
  return r;
}
