// store.js - the robot description shared by every page (one JSON object, kept in localStorage), the sensor
// library, and the conversions robot -> engine config / robot_hardware.txt.
//
// Units in the JSON are what students measure: millimetres, degrees, milliseconds, m/s.
// Frame (same as upstream robot_hardware.txt): origin = midpoint of the drive axle, +x forward, +y LEFT,
// angles counter-clockwise from +x.

import { defaultHardware, normalizeHardware, BOARDS } from './firmware-gen.js';

export const SCHEMA = 'robot-tuner/robot@1';

// ---- sensor library (typical datasheet values; every one can be overridden per sensor) ----------------
// fov_deg: HORIZONTAL full field of view. ToF datasheets give the diagonal FoV of a square receiver:
//   horizontal = 2 atan(tan(diag/2) / sqrt 2)  (VL53L1X 27 deg diag -> 19.3 deg, as in upstream Raycast.hpp).
// model: tof = nearest surface in the cone; ir = narrow beam; ultrasonic = first echo, no echo from walls hit
// more obliquely than max_incidence_deg.
const hfov = (diag) => +(2 * Math.atan(Math.tan((diag / 2) * Math.PI / 180) / Math.SQRT2) * 180 / Math.PI).toFixed(1);
export const SENSOR_LIBRARY = {
  VL53L0X: { model: 'tof', fov_deg: hfov(25), min_mm: 30, max_mm: 1200, period_ms: 33, noise_mm: 5, noise_pct: 1.5, dropout_pct: 1, rays: 3, max_incidence_deg: 0,
    note: 'sensor.note.VL53L0X' },
  VL53L1X_short: { model: 'tof', fov_deg: hfov(27), min_mm: 40, max_mm: 1300, period_ms: 25, noise_mm: 4, noise_pct: 1, dropout_pct: 2, rays: 3, max_incidence_deg: 0,
    note: 'sensor.note.VL53L1X_short' },
  VL53L1X_long: { model: 'tof', fov_deg: hfov(27), min_mm: 40, max_mm: 4000, period_ms: 50, noise_mm: 5, noise_pct: 1, dropout_pct: 2, rays: 3, max_incidence_deg: 0,
    note: 'sensor.note.VL53L1X_long' },
  GP2Y0A21: { model: 'ir', fov_deg: 6, min_mm: 100, max_mm: 800, period_ms: 39, noise_mm: 3, noise_pct: 3, dropout_pct: 0, rays: 1, max_incidence_deg: 0,
    note: 'sensor.note.GP2Y0A21' },
  GP2Y0A41: { model: 'ir', fov_deg: 6, min_mm: 40, max_mm: 300, period_ms: 17, noise_mm: 2, noise_pct: 2, dropout_pct: 0, rays: 1, max_incidence_deg: 0,
    note: 'sensor.note.GP2Y0A41' },
  HC_SR04: { model: 'ultrasonic', fov_deg: 30, min_mm: 20, max_mm: 4000, period_ms: 60, noise_mm: 3, noise_pct: 0.5, dropout_pct: 1, rays: 9, max_incidence_deg: 40,
    note: 'sensor.note.HC_SR04' },
};
export const SENSOR_FIELDS = ['fov_deg', 'min_mm', 'max_mm', 'period_ms', 'noise_mm', 'noise_pct', 'dropout_pct', 'rays', 'max_incidence_deg', 'bias_mm'];
const MODEL_CODE = { tof: 0, ir: 1, ultrasonic: 2 };

export function makeSensor(type, pose = {}) {
  const lib = SENSOR_LIBRARY[type] || SENSOR_LIBRARY.VL53L1X_short;
  const s = { type, name: pose.name || '', x_mm: pose.x_mm ?? 100, y_mm: pose.y_mm ?? 0, angle_deg: pose.angle_deg ?? 0, bias_mm: 0, overrides: {} };
  for (const k of SENSOR_FIELDS) if (k !== 'bias_mm') s[k] = lib[k];
  s.model = lib.model;
  return s;
}

// ---- default robot: Basel's real robot (vendor/simulation-to-real-life/robot_hardware.txt, Nov 2026) ----
export function defaultRobot() {
  return {
    schema: SCHEMA,
    name: 'Example robot (ESP32, 3x VL53L1X)',
    drive: '2wd',
    body: { front_mm: 140, rear_mm: 40, width_mm: 140 },
    wheels: { diameter_mm: 45, width_mm: 20, track_mm: 120, x_mm: 0, wheelbase_mm: 80, track_effective_mm: 124.9,
      ticks_per_rev: 8291.5, scale_l: 0.972, scale_r: 0.972 },
    caster: { enabled: true, x_mm: 105, diameter_mm: 15 },
    motor: {
      model: 'pid',               // 'pid' = firmware wheel PID on a DC-motor model; 'lag' = first-order lag only
      max_speed_mps: 0.156, step_tau_s: 0.10,
      pwm_max: 255, pwm_limit: 255, battery_v: 8.05, nominal_v: 8.05,
      k_l: 0.000613, k_r: 0.000618, static_l: 3.2, static_r: 3.8, breakaway_l: 20, breakaway_r: 20,
      tau_l_s: 0.0362, tau_r_s: 0.04,
      wheel_pid_period_ms: 10, wheel_kp: 2, wheel_ki: 0.001, wheel_kd: 5, ff_static: 4, ff_gain: 2.785,
      v_min_mps: 0.013,           // slowest steady wheel speed (Measure -> Slowest speed); upstream cs_v_min_mps (CAL FF: PWM 25)
    },
    sensors: [
      Object.assign(makeSensor('VL53L1X_short', { name: 'right', x_mm: 100.76, y_mm: -10.24, angle_deg: -45 }), { noise_mm: 4 }),
      Object.assign(makeSensor('VL53L1X_short', { name: 'front', x_mm: 105, y_mm: 0, angle_deg: 0 }), { bias_mm: 7 }),
      Object.assign(makeSensor('VL53L1X_short', { name: 'left', x_mm: 98.64, y_mm: 8.12, angle_deg: 45 }), { noise_mm: 4 }),
    ],
    timing: { loop_ms: 25, age_ms: 22.2, age_jitter_ms: 18 },
    // Constant cruise speed = upstream PidController.hpp const_speed (cs_speed_frac 0.75): base = v_min + speed_frac (v_max - v_min),
    // turn clamped to +-v_max, v_left = base - turn, v_right = base + turn, each clamped to +-v_max
    // (v_max = motor.max_speed_mps, v_min = motor.v_min_mps). No slowdown, no speed law.
    pid: { kp: 0.586855, ki: 0.0158843, kd: 0.384207, speed_frac: 0.75, error_clip_mm: 80, integral_limit: 0.5, d_filter_alpha: 0.3,
      mode: 'two_sides', follow_side: 'left', follow_mm: 100, offset_mm: 0 },
    smoothing: { on: true, tau_s: 0.10, turn_accel: 1.5, lin_accel: 0.5, max_step: 0.0105 },
    // wiring for the generated Arduino sketch (robot builder -> Firmware; see firmware-gen.js)
    hardware: defaultHardware(),
    source: {
      'wheels.ticks_per_rev': 'MEASURED', 'wheels.track_effective_mm': 'MEASURED', 'motor.max_speed_mps': 'MEASURED',
      'motor.ff_gain': 'MEASURED', 'motor.step_tau_s': 'MEASURED', 'timing.loop_ms': 'MEASURED', 'timing.age_ms': 'MEASURED',
    },
  };
}

// ---- persistence ---------------------------------------------------------------------------------
const KEY = 'rt.robot';
export function loadRobot() {
  try { const s = localStorage.getItem(KEY); if (s) return migrate(JSON.parse(s)); } catch (e) { /* ignore */ }
  return defaultRobot();
}
export function saveRobot(r) {
  try { localStorage.setItem(KEY, JSON.stringify(r)); } catch (e) { /* ignore */ }
  window.dispatchEvent(new CustomEvent('robot-changed', { detail: r }));
}
/** Fill any missing field from the defaults (older files, hand-written files). Throws on a foreign file. */
export function migrate(r) {
  if (!r || typeof r !== 'object') throw new Error('not a robot');
  if (r.schema && r.schema !== SCHEMA) throw new Error('unknown schema ' + r.schema);
  const d = defaultRobot();
  const out = { ...d, ...r, schema: SCHEMA };
  for (const k of ['body', 'wheels', 'caster', 'motor', 'timing', 'pid', 'smoothing']) out[k] = { ...d[k], ...(r[k] || {}) };
  out.sensors = (Array.isArray(r.sensors) && r.sensors.length ? r.sensors : d.sensors).slice(0, 5).map((s) => ({ ...makeSensor(s.type || 'VL53L1X_short'), ...s }));
  out.hardware = normalizeHardware(r.hardware, out.sensors.length);
  // before the constant-speed PID: cruise speed in m/s with a slowdown rule. Both are gone; speed_frac keeps its default.
  delete out.pid.speed_mps; delete out.pid.slowdown_frac;
  out.source = { ...(r.source || {}) };
  return out;
}

/** The constant cruise speed (m/s): base = v_min + speed_frac (v_max - v_min). */
export function cruiseSpeed(r, speedFrac = r.pid.speed_frac) {
  const vmax = r.motor.max_speed_mps, vmin = Math.min(Math.max(0, r.motor.v_min_mps || 0), vmax);
  return vmin + speedFrac * (vmax - vmin);
}

export function loopPeriodMs(r) {
  if (r.timing && r.timing.loop_ms > 0) return r.timing.loop_ms;
  return Math.max(...r.sensors.map((s) => s.period_ms || 25));
}

// ---- robot -> engine config (key value object; see sim/rt_engine.hpp ApplyKey) ----------------------
// sim: optional simulation settings {noise: multiplier, sensorNoise, encoderNoise, mismatch, delayMs, ...}
export function robotToConfig(r, sim = {}) {
  const mm = (x) => x / 1000;
  const n = sim.noise ?? 1;  // global noise multiplier (0x / 1x / 2x)
  const sN = (sim.sensorNoise ?? 1) * n, eN = (sim.encoderNoise ?? 1) * n;
  const m = r.motor, w = r.wheels;
  const dt = loopPeriodMs(r) / 1000;
  const c = {
    dt,
    track_width_m: mm(w.track_effective_mm || w.track_mm),
    wheel_track_geom_m: mm(w.track_mm),
    wheel_radius_m: mm(w.diameter_mm) / 2,
    max_linear_speed_mps: m.max_speed_mps,
    chassis_front_m: mm(r.body.front_mm), chassis_rear_m: mm(r.body.rear_mm), chassis_width_m: mm(r.body.width_mm),
    chassis_length_m: mm(r.body.front_mm + r.body.rear_mm),
    wheel_offset_from_center_m: -mm(w.x_mm || 0),
    wheel_length_m: mm(w.diameter_mm), wheel_width_m: mm(w.width_mm),
    four_wheel: r.drive === '4wd' ? 1 : 0, wheel_base_m: r.drive === '4wd' ? mm(w.wheelbase_mm) : 0,
    n_sensors: r.sensors.length,
    enc_ticks_per_rev: w.ticks_per_rev,
    // timing: reading age = latency + uniform jitter
    tof_latency_s: Math.max(0, (r.timing.age_ms - r.timing.age_jitter_ms / 2) / 1000) + (sim.delayMs || 0) / 1000,
    tof_latency_jitter_s: (r.timing.age_jitter_ms / 1000) * Math.min(1, n),
    tof_glitch_threshold_m: sim.glitchThreshold ?? 0.12, tof_glitch_max_hold_s: sim.glitchHold ?? 2.0,
    encoder_model: 1,
    encoder_noise_base: 0.005 * eN, encoder_noise_speed_coeff: 0.02 * eN,
    wheel_slip_coefficient: 0.02 * Math.min(n, 2),
    sensor_angle_bias_std_deg: (sim.angleBias ?? 2.0) * n,
    wheel_scale_bias_std: 0.02 * n, max_speed_bias_std: 0.05 * n,
    // wheel mismatch slider (fraction, e.g. 0.04 = right wheel 4 % faster than left); the encoders cannot see it
    wheel_scale_l: 1 - (sim.mismatch || 0) / 2,
    wheel_scale_r: 1 + (sim.mismatch || 0) / 2,
    emerg_on: 0,
    steer_smooth: r.smoothing.on ? 1 : 0, steer_tau_s: r.smoothing.tau_s, steer_turn_accel: r.smoothing.turn_accel,
    steer_lin_accel: r.smoothing.lin_accel, steer_max_step: r.smoothing.max_step * dt / 0.025,
    kp: r.pid.kp, ki: r.pid.ki, kd: r.pid.kd,
    // constant cruise speed (sim/rt_engine.hpp Pid, pid_speed_mode 1)
    pid_speed_mode: 1, pid_speed_frac: r.pid.speed_frac, pid_v_min: Math.min(Math.max(0, m.v_min_mps || 0), m.max_speed_mps),
    pid_error_clip_m: mm(r.pid.error_clip_mm), pid_integral_limit: r.pid.integral_limit,
    pid_d_filter_alpha: r.pid.d_filter_alpha,
    pid_mode: r.pid.mode === 'one_side' ? 1 : 0, follow_side: r.pid.follow_side === 'right' ? -1 : 1, follow_dist_m: mm(r.pid.follow_mm), center_offset_m: mm(r.pid.offset_mm || 0),
    wall_noise_std_m: 0.01 * n,
  };
  if (m.model === 'pid') {
    Object.assign(c, {
      wheel_model: 1, sim_substeps: 10, battery_v: m.battery_v, motor_nominal_v: m.nominal_v, motor_mismatch_std: 0.02 * n,
      pwm_limit: m.pwm_limit, pwm_max: m.pwm_max, wheel_pid_period_ms: m.wheel_pid_period_ms,
      wheel_kp: m.wheel_kp, wheel_ki: m.wheel_ki, wheel_kd: m.wheel_kd, wheel_ff_static: m.ff_static, wheel_ff_gain: m.ff_gain,
      motor_k_l: m.k_l, motor_k_r: m.k_r, motor_static_l: m.static_l, motor_static_r: m.static_r,
      motor_breakaway_l: m.breakaway_l, motor_breakaway_r: m.breakaway_r, motor_tau_l_s: m.tau_l_s, motor_tau_r_s: m.tau_r_s,
    });
  } else {
    Object.assign(c, { wheel_model: 0, motor_tau_s: m.step_tau_s, motor_max_accel_mps2: 0 });
  }
  r.sensors.forEach((s, i) => {
    const p = `sensor_${i}_`;
    c[p + 'pose'] = [mm(s.x_mm), mm(s.y_mm), s.angle_deg];
    c[p + 'model'] = MODEL_CODE[s.model] ?? 0;
    c[p + 'half_angle_rad'] = (s.fov_deg / 2) * Math.PI / 180;
    c[p + 'rays'] = s.rays;
    c[p + 'max_range_m'] = mm(s.max_mm);
    c[p + 'min_range_m'] = mm(s.min_mm);
    c[p + 'noise_std_m'] = mm(s.noise_mm) * sN;
    c[p + 'noise_pct'] = (s.noise_pct / 100) * sN;
    c[p + 'dropout'] = (s.dropout_pct / 100) * Math.min(n, 3);
    c[p + 'bias_m'] = sim.sensorBias ? mm(s.bias_mm || 0) : 0;
    c[p + 'max_incidence_deg'] = s.max_incidence_deg || 0;
  });
  return c;
}

// ---- robot_hardware.txt (upstream format; tools/gen_hardware.py reads it) ----------------------------
export function robotToHardwareTxt(r) {
  const L = [];
  const src = (k, def = 'TYPED') => (r.source && r.source[k]) || def;
  const line = (k, v, tag, c = '') => L.push(`${k.padEnd(24)} ${String(v).padStart(10)}   # ${tag}${c ? '  ' + c : ''}`);
  L.push('# robot_hardware.txt - written by Robot Tuner (https://github.com/Basel-Shokha/Robot-Tuner)');
  L.push(`# robot: ${r.name}`);
  L.push('# Frame: origin = midpoint of the drive axle, +x forward, +y LEFT, angles CCW (deg), lengths in mm.');
  L.push('');
  const hw = r.hardware ? normalizeHardware(r.hardware, r.sensors.length) : null;
  if (hw) {
    const pinNames = r.sensors.map((s, i) => (r.sensors.length === 3 ? ['right', 'front', 'left'][i] : (s.name || 's' + i)));
    L.push('# Board / pins (Robot builder -> Firmware)');
    L.push(`board                    ${hw.board.padStart(10)}   # TYPED  ${BOARDS[hw.board].label}`);
    L.push(`motor_driver             ${hw.driver.type.padStart(10)}   # TYPED  in1_in2 | pwm_dir | en_in1_in2`);
    L.push(`encoder_type             ${hw.encoder.type.padStart(10)}   # TYPED  quadrature | single | as5600 | none`);
    // upstream names: *_pwm = PWM / IN1, *_dir = DIR / IN2
    for (const [side, m] of [['right', hw.driver.right], ['left', hw.driver.left]]) {
      line(`pin_motor_${side}_pwm`, m.a, 'TYPED', hw.driver.type === 'pwm_dir' ? 'PWM' : 'IN1');
      line(`pin_motor_${side}_dir`, m.b, 'TYPED', hw.driver.type === 'pwm_dir' ? 'DIR' : 'IN2');
      if (hw.driver.type === 'en_in1_in2') line(`pin_motor_${side}_en`, m.en, 'TYPED');
    }
    line('motor_right_sign', hw.driver.sign_r, 'TYPED'); line('motor_left_sign', hw.driver.sign_l, 'TYPED');
    L.push(`motor_stop               ${hw.driver.stop.padStart(10)}   # TYPED  brake | coast`);
    if (hw.driver.stby >= 0) line('pin_motor_stby', hw.driver.stby, 'TYPED');
    line('enc_ppr', hw.encoder.ppr, 'TYPED', 'pulses (or slots) per motor turn, per channel');
    line('enc_gear', hw.encoder.gear, 'TYPED', 'motor (or magnet) turns per wheel turn');
    L.push(`enc_decoding             ${hw.encoder.avr_mode.padStart(10)}   # TYPED  x1 | x4 (Uno / Mega quadrature)`);
    L.push(`as5600_bus               ${hw.encoder.as5600_bus.padStart(10)}   # TYPED  mux | two_buses`);
    line('as5600_mux_addr', '0x' + (+hw.encoder.mux_addr).toString(16), 'TYPED'); line('as5600_mux_ch_left', hw.encoder.mux_ch_l, 'TYPED'); line('as5600_mux_ch_right', hw.encoder.mux_ch_r, 'TYPED');
    line('pin_i2c2_sda', hw.encoder.sda2, 'TYPED'); line('pin_i2c2_scl', hw.encoder.scl2, 'TYPED');
    for (const [side, c] of [['right', hw.encoder.right], ['left', hw.encoder.left]]) { line(`pin_enc_${side}_a`, c.a, 'TYPED'); line(`pin_enc_${side}_b`, c.b, 'TYPED'); }
    line('enc_right_sign', hw.encoder.sign_r, 'TYPED'); line('enc_left_sign', hw.encoder.sign_l, 'TYPED');
    line('pin_i2c_sda', hw.i2c.sda, 'TYPED'); line('pin_i2c_scl', hw.i2c.scl, 'TYPED');
    hw.sensors.forEach((p, i) => {
      if (p.xshut >= 0) line(`pin_xshut_${pinNames[i]}`, p.xshut, 'TYPED');
      if (p.xshut >= 0) line(`tof_addr_${pinNames[i]}`, '0x' + (+p.addr).toString(16), 'TYPED');
      if (p.trig >= 0) line(`pin_trig_${pinNames[i]}`, p.trig, 'TYPED');
      if (p.echo >= 0) line(`pin_echo_${pinNames[i]}`, p.echo, 'TYPED');
      if (p.analog >= 0) line(`pin_analog_${pinNames[i]}`, p.analog, 'TYPED');
    });
    L.push('');
  }
  L.push('# Geometry (mm)');
  line('wheel_diameter_mm', r.wheels.diameter_mm, src('wheels.diameter_mm'));
  line('wheel_width_mm', r.wheels.width_mm, src('wheels.width_mm'));
  line('wheel_track_mm', r.wheels.track_mm, src('wheels.track_mm'), 'wheel centre to centre');
  line('wheel_x_mm', r.wheels.x_mm, src('wheels.x_mm'));
  line('body_front_mm', r.body.front_mm, src('body.front_mm'));
  line('body_rear_mm', r.body.rear_mm, src('body.rear_mm'));
  line('body_width_mm', r.body.width_mm, src('body.width_mm'));
  if (r.caster.enabled) { line('caster_x_mm', r.caster.x_mm, src('caster.x_mm')); line('caster_diameter_mm', r.caster.diameter_mm, src('caster.diameter_mm')); }
  line('track_w_effective_mm', r.wheels.track_effective_mm, src('wheels.track_effective_mm'), 'CAL SPIN');
  L.push(`# drive ${r.drive}${r.drive === '4wd' ? ` (skid steer), wheelbase ${r.wheels.wheelbase_mm} mm` : ''}`);
  L.push('');
  L.push('# Sensors: explicit poses "x_mm y_mm angle_deg" (Robot Tuner writes one line per sensor)');
  const names = ['right', 'front', 'left'];
  r.sensors.forEach((s, i) => {
    const nm = r.sensors.length === 3 ? names[i] : (s.name || 's' + i);
    L.push(`tof_pose_${nm.padEnd(14)} ${s.x_mm.toFixed(2)} ${s.y_mm.toFixed(2)} ${s.angle_deg}   # ${src('sensors.' + i, 'TYPED')}  ${s.type}`);
  });
  r.sensors.forEach((s, i) => {
    const nm = r.sensors.length === 3 ? names[i] : (s.name || 's' + i);
    line(`tof_bias_${nm}_mm`, s.bias_mm || 0, src(`sensors.${i}.bias_mm`, 'PLACEHOLDER'));
  });
  const s0 = r.sensors[0];
  line('tof_max_range_mm', s0.max_mm, 'DATASHEET', s0.type);
  line('tof_min_range_mm', s0.min_mm, 'DATASHEET');
  line('tof_period_ms', Math.max(...r.sensors.map((s) => s.period_ms)), src('timing.loop_ms', 'DATASHEET'));
  L.push('');
  L.push('# Drive train');
  line('enc_ticks_per_rev', r.wheels.ticks_per_rev, src('wheels.ticks_per_rev'), 'CAL TICKS');
  line('wheel_scale_left', r.wheels.scale_l, src('wheels.scale_l'), 'CAL DRIVE');
  line('wheel_scale_right', r.wheels.scale_r, src('wheels.scale_r'), 'CAL DRIVE');
  line('max_speed_mps', r.motor.max_speed_mps, src('motor.max_speed_mps'), 'CAL MAXSPEED');
  line('v_min_mps', r.motor.v_min_mps || 0, src('motor.v_min_mps', 'PLACEHOLDER'), 'CAL VMIN: slowest steady wheel speed');
  line('pid_speed_frac', r.pid.speed_frac, src('pid.speed_frac', 'TYPED'), 'cruise = v_min + speed_frac (v_max - v_min)');
  line('pwm_limit', r.motor.pwm_limit, src('motor.pwm_limit'));
  line('wheel_pid_period_ms', r.motor.wheel_pid_period_ms, src('motor.wheel_pid_period_ms', 'ASSUMED'));
  const r50 = r.motor.wheel_pid_period_ms / 50;  // inverse of wheel_pid.h ConvertGains (50 ms -> period)
  line('wheel_kp_50ms', +(r.motor.wheel_kp * r50).toPrecision(4), src('motor.wheel_kp'));
  line('wheel_ki_50ms', r.motor.wheel_ki, src('motor.wheel_ki'));
  line('wheel_kd_50ms', +(r.motor.wheel_kd * r50 * r50).toPrecision(4), src('motor.wheel_kd'));
  line('wheel_ff_gain_50ms', +(r.motor.ff_gain * r50).toPrecision(4), src('motor.ff_gain'), 'CAL FF');
  line('wheel_ff_static', r.motor.ff_static, src('motor.ff_static'), 'CAL FF');
  line('motor_k_left', r.motor.k_l, src('motor.k_l'));
  line('motor_k_right', r.motor.k_r, src('motor.k_r'));
  line('motor_static_left', r.motor.static_l, src('motor.static_l'));
  line('motor_static_right', r.motor.static_r, src('motor.static_r'));
  line('motor_breakaway_left', r.motor.breakaway_l, src('motor.breakaway_l'));
  line('motor_breakaway_right', r.motor.breakaway_r, src('motor.breakaway_r'));
  line('motor_tau_left_s', r.motor.tau_l_s, src('motor.tau_l_s'));
  line('motor_tau_right_s', r.motor.tau_r_s, src('motor.tau_r_s'));
  line('motor_nominal_v', r.motor.nominal_v, src('motor.nominal_v', 'ASSUMED'));
  L.push('');
  L.push('# Control timing');
  line('control_period_ms', loopPeriodMs(r), src('timing.loop_ms', 'DERIVED'), 'measured loop period');
  line('reading_age_ms', r.timing.age_ms, src('timing.age_ms', 'ASSUMED'), 'Robot Tuner key');
  line('reading_age_jitter_ms', r.timing.age_jitter_ms, src('timing.age_jitter_ms', 'ASSUMED'), 'Robot Tuner key');
  return L.join('\n') + '\n';
}

/** Read the keys of a robot_hardware.txt (ours or upstream's) into a robot (missing keys keep `base`). */
export function hardwareTxtToRobot(text, base = defaultRobot()) {
  const r = JSON.parse(JSON.stringify(base));
  const kv = {}, words = {};
  const poses = [];
  for (const raw of text.split('\n')) {
    const line = raw.replace(/#.*/, '').trim();
    if (!line) continue;
    const [k, ...v] = line.split(/\s+/);
    if (k.startsWith('tof_pose_') && v.length >= 3) poses.push({ name: k.slice(9), x_mm: +v[0], y_mm: +v[1], angle_deg: +v[2] });
    else { kv[k] = +v[0]; words[k] = v[0]; }
  }
  const set = (key, obj, field, f = (x) => x) => { if (kv[key] !== undefined && !Number.isNaN(kv[key])) { obj[field] = f(kv[key]); r.source[`${objName(obj, r)}.${field}`] = 'IMPORTED'; } };
  set('wheel_diameter_mm', r.wheels, 'diameter_mm'); set('wheel_width_mm', r.wheels, 'width_mm'); set('wheel_track_mm', r.wheels, 'track_mm');
  set('wheel_x_mm', r.wheels, 'x_mm'); set('body_front_mm', r.body, 'front_mm'); set('body_rear_mm', r.body, 'rear_mm'); set('body_width_mm', r.body, 'width_mm');
  set('caster_x_mm', r.caster, 'x_mm'); set('caster_diameter_mm', r.caster, 'diameter_mm');
  set('track_w_effective_mm', r.wheels, 'track_effective_mm'); set('enc_ticks_per_rev', r.wheels, 'ticks_per_rev');
  set('wheel_scale_left', r.wheels, 'scale_l'); set('wheel_scale_right', r.wheels, 'scale_r');
  set('max_speed_mps', r.motor, 'max_speed_mps'); set('v_min_mps', r.motor, 'v_min_mps');
  if (kv.pid_speed_frac !== undefined && Number.isFinite(kv.pid_speed_frac)) r.pid.speed_frac = kv.pid_speed_frac; set('pwm_limit', r.motor, 'pwm_limit'); set('wheel_pid_period_ms', r.motor, 'wheel_pid_period_ms');
  const r50 = (kv.wheel_pid_period_ms || r.motor.wheel_pid_period_ms) / 50;
  set('wheel_kp_50ms', r.motor, 'wheel_kp', (x) => x / r50); set('wheel_ki_50ms', r.motor, 'wheel_ki');
  set('wheel_kd_50ms', r.motor, 'wheel_kd', (x) => x / (r50 * r50)); set('wheel_ff_gain_50ms', r.motor, 'ff_gain', (x) => x / r50);
  set('wheel_ff_static', r.motor, 'ff_static');
  set('motor_k_left', r.motor, 'k_l'); set('motor_k_right', r.motor, 'k_r'); set('motor_static_left', r.motor, 'static_l'); set('motor_static_right', r.motor, 'static_r');
  set('motor_breakaway_left', r.motor, 'breakaway_l'); set('motor_breakaway_right', r.motor, 'breakaway_r');
  set('motor_tau_left_s', r.motor, 'tau_l_s'); set('motor_tau_right_s', r.motor, 'tau_r_s'); set('motor_nominal_v', r.motor, 'nominal_v');
  set('control_period_ms', r.timing, 'loop_ms'); set('reading_age_ms', r.timing, 'age_ms'); set('reading_age_jitter_ms', r.timing, 'age_jitter_ms');
  if (poses.length) {
    // upstream order is right, front, left; keep the file's order otherwise
    const typ = base.sensors[0] ? base.sensors[0].type : 'VL53L1X_short';
    r.sensors = poses.slice(0, 5).map((p, i) => {
      const s = { ...(base.sensors[i] || makeSensor(typ)), ...p };
      const b = kv[`tof_bias_${p.name}_mm`];
      if (b !== undefined) s.bias_mm = b;
      return s;
    });
  } else if (kv.tof_front_x_mm !== undefined) {
    // upstream mount description (tools/gen_hardware.py DeriveTofPoses), simplified: three sensors at the front face
    const fx = kv.tof_front_x_mm, face = kv.tof_face_width_mm || 12, ry = kv.tof_right_yaw_deg ?? 45, ly = kv.tof_left_yaw_deg ?? 45;
    const rr = ry * Math.PI / 180, lr = ly * Math.PI / 180, bs = kv.tof_left_backshift_mm || 0;
    const right = { x_mm: fx - Math.sin(rr) * face / 2, y_mm: -face / 2 - Math.cos(rr) * face / 2, angle_deg: -ry };
    const left = { x_mm: fx - Math.sin(lr) * face / 2 - Math.cos(lr) * bs, y_mm: face / 2 + Math.cos(lr) * face / 2 - Math.sin(lr) * bs, angle_deg: ly };
    const front = { x_mm: fx, y_mm: 0, angle_deg: 0 };
    [right, front, left].forEach((p, i) => { r.sensors[i] = { ...(r.sensors[i] || makeSensor('VL53L1X_short')), ...p }; });
  }
  readPins(r, kv, words);
  return migrate(r);
}
// pin_* keys (upstream robot_hardware.txt names) -> robot.hardware
function readPins(r, kv, words) {
  const hw = normalizeHardware(r.hardware, r.sensors.length);
  const has = (k) => kv[k] !== undefined && Number.isFinite(kv[k]);
  if (words.board && BOARDS[words.board]) hw.board = words.board;
  if (['in1_in2', 'pwm_dir', 'en_in1_in2'].includes(words.motor_driver)) hw.driver.type = words.motor_driver;
  if (['quadrature', 'single', 'as5600', 'none'].includes(words.encoder_type)) hw.encoder.type = words.encoder_type;
  for (const side of ['right', 'left']) {
    const m = hw.driver[side], c = hw.encoder[side];
    if (has(`pin_motor_${side}_pwm`)) m.a = kv[`pin_motor_${side}_pwm`];
    if (has(`pin_motor_${side}_dir`)) m.b = kv[`pin_motor_${side}_dir`];
    if (has(`pin_motor_${side}_en`)) m.en = kv[`pin_motor_${side}_en`];
    if (has(`pin_enc_${side}_a`)) c.a = kv[`pin_enc_${side}_a`];
    if (has(`pin_enc_${side}_b`)) c.b = kv[`pin_enc_${side}_b`];
  }
  if (['brake', 'coast'].includes(words.motor_stop)) hw.driver.stop = words.motor_stop;
  if (['x1', 'x4'].includes(words.enc_decoding)) hw.encoder.avr_mode = words.enc_decoding;
  if (['mux', 'two_buses'].includes(words.as5600_bus)) hw.encoder.as5600_bus = words.as5600_bus;
  for (const [key, obj, field] of [['pin_motor_stby', hw.driver, 'stby'], ['enc_ppr', hw.encoder, 'ppr'], ['enc_gear', hw.encoder, 'gear'], ['as5600_mux_addr', hw.encoder, 'mux_addr'],
    ['as5600_mux_ch_left', hw.encoder, 'mux_ch_l'], ['as5600_mux_ch_right', hw.encoder, 'mux_ch_r'], ['pin_i2c2_sda', hw.encoder, 'sda2'], ['pin_i2c2_scl', hw.encoder, 'scl2']]) if (has(key)) obj[field] = kv[key];
  if (has('motor_right_sign')) hw.driver.sign_r = kv.motor_right_sign; if (has('motor_left_sign')) hw.driver.sign_l = kv.motor_left_sign;
  if (has('enc_right_sign')) hw.encoder.sign_r = kv.enc_right_sign; if (has('enc_left_sign')) hw.encoder.sign_l = kv.enc_left_sign;
  if (has('pin_i2c_sda')) hw.i2c.sda = kv.pin_i2c_sda; if (has('pin_i2c_scl')) hw.i2c.scl = kv.pin_i2c_scl;
  const names = r.sensors.map((s, i) => (r.sensors.length === 3 ? ['right', 'front', 'left'][i] : (s.name || 's' + i)));
  names.forEach((n, i) => {
    const p = hw.sensors[i];
    if (has(`pin_xshut_${n}`)) p.xshut = kv[`pin_xshut_${n}`];
    if (has(`tof_addr_${n}`)) p.addr = kv[`tof_addr_${n}`];
    if (has(`pin_trig_${n}`)) p.trig = kv[`pin_trig_${n}`];
    if (has(`pin_echo_${n}`)) p.echo = kv[`pin_echo_${n}`];
    if (has(`pin_analog_${n}`)) p.analog = kv[`pin_analog_${n}`];
  });
  r.hardware = hw;
}
function objName(obj, r) { for (const k of ['wheels', 'body', 'caster', 'motor', 'timing']) if (r[k] === obj) return k; return '?'; }

export function downloadName(r, ext) { return (r.name || 'robot').replace(/[^a-z0-9_-]+/gi, '_').replace(/^_|_$/g, '').toLowerCase() + ext; }
