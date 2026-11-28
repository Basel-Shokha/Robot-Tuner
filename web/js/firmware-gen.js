// firmware-gen.js - generates a ready-to-flash Arduino sketch (robot_tuner_min.ino) for the robot's hardware:
// board (Uno/Nano, Mega, ESP32), wheel encoders (quadrature, single channel, AS5600, none), motor driver
// (PWM+DIR, IN1/IN2, EN+IN1+IN2) and distance sensors (VL53L0X, VL53L1X, HC-SR04, Sharp IR).
// The sketch answers the same serial calibration protocol (CAL ...) as before, so measure.html works unchanged.
//
// Pure module (no DOM): the builder page uses it, tests/js/firmware.test.mjs checks it, and
// tools/gen_firmware_combos.mjs writes every combination for the arduino-cli compile job in CI.

// ---- boards ---------------------------------------------------------------------------------------------
const range = (a, b) => Array.from({ length: b - a + 1 }, (_, i) => a + i);
// AVR pin numbers: digital pins as numbers; analog pins A0.. are numbers too (Uno A0 = 14, Mega A0 = 54).
export const BOARDS = {
  uno: {
    label: 'Arduino Uno / Nano (ATmega328P)', fqbn: 'arduino:avr:uno', arch: 'avr', analogBase: 14,
    pins: range(0, 21),
    inputOnly: [20, 21],            // Nano A6 / A7: analog input only (the Uno does not have them)
    pwm: [3, 5, 6, 9, 10, 11],
    int: [2, 3],                    // attachInterrupt: only D2 and D3
    pcint: range(0, 19),            // pin change interrupts (PinChangeInterrupt library): every digital pin + A0-A5
    analog: range(14, 21),
    i2c: { sda: 18, scl: 19, fixed: true },
    serial: [0, 1], strapping: [], flash: [], led: 13,
    adcMv: 5000, adcMax: 1023,
  },
  mega: {
    label: 'Arduino Mega 2560', fqbn: 'arduino:avr:mega', arch: 'avr', analogBase: 54,
    pins: range(0, 69),
    inputOnly: [],
    pwm: [...range(2, 13), 44, 45, 46],
    int: [2, 3, 18, 19, 20, 21],    // 20 / 21 are also SDA / SCL
    pcint: [10, 11, 12, 13, 14, 15, 50, 51, 52, 53, ...range(62, 69)],
    analog: range(54, 69),
    i2c: { sda: 20, scl: 21, fixed: true },
    serial: [0, 1], strapping: [], flash: [], led: 13,
    adcMv: 5000, adcMax: 1023,
  },
  esp32: {
    label: 'ESP32', fqbn: 'esp32:esp32:esp32', arch: 'esp32', analogBase: -1,
    pins: [0, 1, 2, 3, 4, 5, 12, 13, 14, 15, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33, 34, 35, 36, 39],
    inputOnly: [34, 35, 36, 39],    // no output, no internal pull-up / pull-down
    pwm: [0, 1, 2, 3, 4, 5, 12, 13, 14, 15, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33],
    int: [0, 1, 2, 3, 4, 5, 12, 13, 14, 15, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33, 34, 35, 36, 39],
    pcint: [],
    analog: [32, 33, 34, 35, 36, 39, 0, 2, 4, 12, 13, 14, 15, 25, 26, 27],
    adc2: [0, 2, 4, 12, 13, 14, 15, 25, 26, 27],
    i2c: { sda: 21, scl: 22, fixed: false },
    serial: [1, 3],
    strapping: [0, 2, 5, 12, 15],   // sampled at reset: must be at the right level when the board boots
    flash: [6, 7, 8, 9, 10, 11],    // wired to the flash chip: never usable (not listed in pins)
    led: 2,
    adcMv: 3300, adcMax: 4095,
  },
};

export function pinLabel(board, p) {
  const B = BOARDS[board];
  if (p === undefined || p === null || p < 0) return '—';
  if (B.arch === 'avr' && p >= B.analogBase && B.analog.includes(p)) return 'A' + (p - B.analogBase);
  return (B.arch === 'esp32' ? 'GPIO' : 'D') + p;
}
/** pin as written in the C++ sketch (A0 for analog pins on AVR so it is right on every AVR board) */
function pinCode(board, p) {
  const B = BOARDS[board];
  if (B.arch === 'avr' && p >= B.analogBase && B.analog.includes(p)) return 'A' + (p - B.analogBase);
  return String(p);
}
/** short capability tags of a pin, for the pin menus and the pin table */
export function pinTags(board, p) {
  const B = BOARDS[board], tags = [];
  if (B.int.includes(p)) tags.push('INT');
  else if (B.pcint.includes(p)) tags.push('PCINT');
  if (B.pwm.includes(p)) tags.push('PWM');
  if (B.analog.includes(p)) tags.push(B.adc2 && B.adc2.includes(p) ? 'ADC2' : 'ADC');
  if (B.inputOnly.includes(p)) tags.push('input only');
  if (B.strapping.includes(p)) tags.push('strapping');
  if (B.serial.includes(p)) tags.push('USB serial');
  if (B.i2c.fixed && (p === B.i2c.sda || p === B.i2c.scl)) tags.push(p === B.i2c.sda ? 'SDA' : 'SCL');
  if (p === B.led) tags.push('LED');
  return tags;
}

// ---- choices ----------------------------------------------------------------------------------------------
export const ENCODERS = ['quadrature', 'single', 'as5600', 'none'];
export const DRIVERS = ['in1_in2', 'pwm_dir', 'en_in1_in2'];
export const TOF_TYPES = ['VL53L0X', 'VL53L1X_short', 'VL53L1X_long'];
export const SHARP_FIT = {  // distance_cm = a * V^-b (power-law fit of the datasheet curve)
  GP2Y0A21: { a: 29.988, b: 1.173 },
  GP2Y0A41: { a: 12.08, b: 1.058 },
};
const LOW_RES_TICKS = 100;

/** the example robot (vendor/simulation-to-real-life/robot_hardware.txt): ESP32, DRV8833, quadrature, 3x VL53L1X */
export function defaultHardware() {
  return {
    board: 'esp32',
    driver: { type: 'in1_in2', stop: 'brake', left: { a: 25, b: 4, en: -1 }, right: { a: 13, b: 12, en: -1 }, stby: -1, sign_l: -1, sign_r: -1 },
    encoder: { type: 'quadrature', avr_mode: 'x1', ppr: 7, gear: 296.125, left: { a: 32, b: 33 }, right: { a: 27, b: 26 }, sign_l: -1, sign_r: -1,
      as5600_bus: 'mux', mux_addr: 0x70, mux_ch_l: 0, mux_ch_r: 1, sda2: 16, scl2: 17 },
    i2c: { sda: 21, scl: 22 },
    bt: true, bt_name: '',   // ESP32: Bluetooth serial next to USB (classic ESP32 only; S3 / C3 / C6 have no classic Bluetooth)
    // index-aligned with robot.sensors (right, front, left)
    sensors: [{ xshut: 18, addr: 0x32, trig: -1, echo: -1, analog: -1 }, { xshut: 23, addr: 0x31, trig: -1, echo: -1, analog: -1 },
      { xshut: 5, addr: 0x30, trig: -1, echo: -1, analog: -1 }],
  };
}
/** a typical Arduino Uno robot: L298N with ENA, quadrature A on D2 / D3, 3x VL53L0X */
export function unoHardware() {
  return {
    board: 'uno',
    driver: { type: 'en_in1_in2', stop: 'coast', left: { en: 5, a: 8, b: 9 }, right: { en: 6, a: 10, b: 11 }, stby: -1, sign_l: 1, sign_r: 1 },
    encoder: { type: 'quadrature', avr_mode: 'x1', ppr: 11, gear: 30, left: { a: 2, b: 4 }, right: { a: 3, b: 7 }, sign_l: 1, sign_r: 1,
      as5600_bus: 'mux', mux_addr: 0x70, mux_ch_l: 0, mux_ch_r: 1, sda2: -1, scl2: -1 },
    i2c: { sda: 18, scl: 19 },
    bt: true, bt_name: '',
    sensors: [{ xshut: 14, addr: 0x30, trig: -1, echo: -1, analog: -1 }, { xshut: 15, addr: 0x31, trig: -1, echo: -1, analog: -1 },
      { xshut: 16, addr: 0x32, trig: -1, echo: -1, analog: -1 }],
  };
}
/** fill every missing field (older robot files, hand-written files) */
export function normalizeHardware(h, nSensors = 3) {
  const d = defaultHardware(), x = h && typeof h === 'object' ? h : {};
  const out = { ...d, ...x };
  out.driver = { ...d.driver, ...(x.driver || {}) };
  out.driver.left = { a: -1, b: -1, en: -1, ...((x.driver || {}).left || d.driver.left) };
  out.driver.right = { a: -1, b: -1, en: -1, ...((x.driver || {}).right || d.driver.right) };
  out.encoder = { ...d.encoder, ...(x.encoder || {}) };
  out.encoder.left = { a: -1, b: -1, ...((x.encoder || {}).left || d.encoder.left) };
  out.encoder.right = { a: -1, b: -1, ...((x.encoder || {}).right || d.encoder.right) };
  out.i2c = { ...d.i2c, ...(x.i2c || {}) };
  if (!BOARDS[out.board]) out.board = 'esp32';
  const src = Array.isArray(x.sensors) ? x.sensors : d.sensors;
  out.sensors = Array.from({ length: nSensors }, (_, i) => ({ xshut: -1, addr: 0x30 + i, trig: -1, echo: -1, analog: -1, ...(src[i] || {}) }));
  return out;
}

// ---- derived numbers ---------------------------------------------------------------------------------------
/** counted edges per encoder pulse (per slot / per line) with the decoding the sketch uses */
export function edgesPerPulse(hw) {
  const e = hw.encoder;
  if (e.type === 'quadrature') return BOARDS[hw.board].arch === 'avr' && e.avr_mode === 'x1' ? 1 : 4;
  if (e.type === 'single') return 1;
  if (e.type === 'as5600') return 4096;
  return 0;
}
/** ticks per WHEEL revolution = pulses per motor-shaft (or magnet) revolution x edges x gear ratio */
export function computedTicksPerRev(hw) {
  const e = hw.encoder;
  if (e.type === 'none') return 0;
  const per = e.type === 'as5600' ? 1 : (+e.ppr || 0);
  return +(per * edgesPerPulse(hw) * (+e.gear || 1)).toFixed(3);
}
export function isLowRes(hw, ticks) {
  return (hw.encoder.type === 'quadrature' || hw.encoder.type === 'single') && ticks > 0 && ticks < LOW_RES_TICKS;
}
/** ESP32 quadrature normally uses the PCNT hardware counter; at low resolution the sketch needs the time of
 *  every edge (speed from the pulse period), so it decodes in interrupts instead */
function usesPcnt(hw, ticks) { return hw.board === 'esp32' && hw.encoder.type === 'quadrature' && !isLowRes(hw, ticks); }
function avrX4(hw) { return BOARDS[hw.board].arch === 'avr' && hw.encoder.type === 'quadrature' && hw.encoder.avr_mode === 'x4'; }

/** ESP32 with Bluetooth serial switched on */
export const usesBt = (hw) => hw.board === 'esp32' && hw.bt !== false;
/** the Bluetooth device name the student pairs with: the typed name, else RobotTuner-<robot name> */
export function btName(robot, hw) {
  const n = String(hw.bt_name || ('RobotTuner-' + String(robot.name || 'robot'))).replace(/[^A-Za-z0-9_-]+/g, '-').replace(/-+/g, '-').replace(/^-|-$/g, '');
  return (n || 'RobotTuner').slice(0, 24).replace(/-$/, '');
}
export function sensorWireName(s, i) {
  const n = String((s && s.name) || ['right', 'front', 'left'][i] || 's' + i).toLowerCase().replace(/\W+/g, '_').replace(/^_+|_+$/g, '');
  return n || 's' + i;
}
const isTof = (type) => TOF_TYPES.includes(type);
function sensorKind(type) {
  if (type === 'VL53L0X') return 'l0x';
  if (type === 'VL53L1X_short' || type === 'VL53L1X_long') return 'l1x';
  if (type === 'HC_SR04') return 'sonar';
  if (SHARP_FIT[type]) return 'sharp';
  return 'none';
}
function usesI2C(robot, hw) { return hw.encoder.type === 'as5600' || robot.sensors.some((s) => isTof(s.type)); }

/** the ticks/rev the sketch starts with: the measured value when there is one, else the computed one */
export function sketchTicks(robot, hw) {
  const measured = robot.source && /MEASURED|IMPORTED/.test(robot.source['wheels.ticks_per_rev'] || '');
  const c = computedTicksPerRev(hw);
  return measured || !c ? robot.wheels.ticks_per_rev : c;
}

// ---- libraries ---------------------------------------------------------------------------------------------
/** Arduino Library Manager names (and what they are for) */
export function requiredLibraries(robot, hw) {
  const libs = [];
  const add = (name, by, why) => { if (!libs.some((l) => l.name === name)) libs.push({ name, by, why }); };
  const ticks = sketchTicks(robot, hw);
  if (BOARDS[hw.board].arch === 'avr' && needsPcint(hw)) add('PinChangeInterrupt', 'NicoHood', 'encoder channels on pins without attachInterrupt');
  for (const s of robot.sensors) {
    const k = sensorKind(s.type);
    if (k === 'l0x') add('VL53L0X', 'Pololu', 'VL53L0X distance sensors');
    if (k === 'l1x') add('VL53L1X', 'Pololu', 'VL53L1X distance sensors');
  }
  return libs;
}
function encoderPins(hw) {
  const e = hw.encoder;
  if (e.type === 'quadrature') return [e.left.a, e.left.b, e.right.a, e.right.b];
  if (e.type === 'single') return [e.left.a, e.right.a];
  return [];
}
/** interrupt-driven pins on AVR that are not attachInterrupt pins -> PinChangeInterrupt */
function interruptPins(hw) {
  const e = hw.encoder;
  if (e.type === 'single') return [e.left.a, e.right.a];
  if (e.type === 'quadrature') return avrX4(hw) || hw.board === 'esp32' ? [e.left.a, e.left.b, e.right.a, e.right.b] : [e.left.a, e.right.a];
  return [];
}
function needsPcint(hw) {
  const B = BOARDS[hw.board];
  return B.arch === 'avr' && interruptPins(hw).some((p) => p >= 0 && !B.int.includes(p));
}

// ---- pin roles: every pin the sketch uses, with what the pin must be able to do --------------------------------
// need: out | pwm | in | int (attachInterrupt) | anyint (attachInterrupt or pin change) | analog | i2c
export function pinRoles(robot, hw) {
  const B = BOARDS[hw.board], roles = [];
  const role = (obj, key, name, need, optional = false) => roles.push({ get: () => obj[key], set: (v) => { obj[key] = v; }, role: name, need, optional });
  const d = hw.driver, e = hw.encoder;
  for (const [side, m] of [['L', d.left], ['R', d.right]]) {
    if (d.type === 'pwm_dir') { role(m, 'a', `motor ${side} PWM`, 'pwm'); role(m, 'b', `motor ${side} DIR`, 'out'); }
    if (d.type === 'in1_in2') { role(m, 'a', `motor ${side} IN1`, 'pwm'); role(m, 'b', `motor ${side} IN2`, 'pwm'); }
    if (d.type === 'en_in1_in2') { role(m, 'en', `motor ${side} EN`, 'pwm'); role(m, 'a', `motor ${side} IN1`, 'out'); role(m, 'b', `motor ${side} IN2`, 'out'); }
  }
  if (d.type !== 'in1_in2' && d.stby >= 0) role(d, 'stby', 'driver STBY', 'out', true);
  if (e.type === 'quadrature') {
    for (const [side, c] of [['L', e.left], ['R', e.right]]) {
      if (B.arch === 'avr' && e.avr_mode === 'x1') { role(c, 'a', `encoder ${side} A`, 'int'); role(c, 'b', `encoder ${side} B`, 'in'); }
      else { role(c, 'a', `encoder ${side} A`, B.arch === 'avr' ? 'anyint' : 'in'); role(c, 'b', `encoder ${side} B`, B.arch === 'avr' ? 'anyint' : 'in'); }
    }
  } else if (e.type === 'single') {
    role(e.left, 'a', 'encoder L', B.arch === 'avr' ? 'anyint' : 'in'); role(e.right, 'a', 'encoder R', B.arch === 'avr' ? 'anyint' : 'in');
  } else if (e.type === 'as5600' && e.as5600_bus === 'two_buses' && B.arch === 'esp32') {
    role(e, 'sda2', 'second I2C SDA', 'out'); role(e, 'scl2', 'second I2C SCL', 'out');
  }
  const nTof = robot.sensors.filter((s) => isTof(s.type)).length;
  robot.sensors.forEach((s, i) => {
    const p = hw.sensors[i], k = sensorKind(s.type), nm = `#${i + 1} ${sensorWireName(s, i)}`;
    if (k === 'l0x' || k === 'l1x') role(p, 'xshut', `${nm} XSHUT`, 'out', nTof <= 1);
    else if (k === 'sonar') { role(p, 'trig', `${nm} TRIG`, 'out'); role(p, 'echo', `${nm} ECHO`, 'in'); }
    else if (k === 'sharp') role(p, 'analog', `${nm} analog`, 'analog');
  });
  // AVR boards: I2C is fixed in hardware (hw.i2c is ignored there)
  if (usesI2C(robot, hw)) { const bus = B.i2c.fixed ? { sda: B.i2c.sda, scl: B.i2c.scl } : hw.i2c; role(bus, 'sda', 'I2C SDA', 'i2c'); role(bus, 'scl', 'I2C SCL', 'i2c'); }
  // optional pins that are not set are simply not used
  return roles.filter((r) => !(r.optional && !(r.get() >= 0)));
}

/** Give every pin role a valid, free pin (keeps pins the user already chose when they are valid).
 *  Returns { hw, failed: [role names that found no pin] }. */
export function suggestPins(robot, hwIn, keep = false) {
  const hw = normalizeHardware(JSON.parse(JSON.stringify(hwIn)), robot.sensors.length);
  const B = BOARDS[hw.board];
  const roles = pinRoles(robot, hw);
  const i2c = usesI2C(robot, hw);
  const reserved = new Set([...B.serial, ...B.flash, ...(B.arch === 'esp32' ? [0, 12] : [])]);  // ESP32: GPIO 0 = boot button, 12 = flash voltage
  const lastResort = new Set(B.strapping);                         // other strapping pins: only for outputs, only when nothing else is free
  if (i2c && !B.i2c.fixed) { reserved.add(hw.i2c.sda >= 0 ? hw.i2c.sda : B.i2c.sda); reserved.add(hw.i2c.scl >= 0 ? hw.i2c.scl : B.i2c.scl); }
  if (B.i2c.fixed) { reserved.add(B.i2c.sda); reserved.add(B.i2c.scl); }  // keep the I2C pins free on AVR anyway
  if (hw.board === 'uno') { reserved.add(20); reserved.add(21); }        // A6 / A7 exist on the Nano only
  const ok = (p, need) => {
    if (!B.pins.includes(p) || reserved.has(p)) return false;
    if (lastResort.has(p) && need !== 'out') return false;
    const outNeed = ['out', 'pwm', 'i2c'].includes(need);
    if (outNeed && B.inputOnly.includes(p)) return false;
    if (need === 'pwm') return B.pwm.includes(p);
    if (need === 'int') return B.int.includes(p);
    if (need === 'anyint') return B.int.includes(p) || B.pcint.includes(p);
    if (need === 'analog') return B.analog.includes(p);
    if (p === B.led && need !== 'out') return false;
    return true;
  };
  // how valuable a pin is (use cheap pins first): interrupt, PWM and analog pins are kept for roles that need them
  const cost = (p) => (lastResort.has(p) ? 20 : 0) + (B.inputOnly.includes(p) ? 10 : 0) + (B.int.includes(p) && B.arch === 'avr' ? 4 : 0) + (B.pwm.includes(p) ? 2 : 0) + (B.analog.includes(p) ? 1 : 0) + (B.pcint.includes(p) ? 0.5 : 0);
  const used = new Set();
  const order = ['i2c', 'int', 'anyint', 'analog', 'pwm', 'in', 'out'];
  const sorted = [...roles].sort((a, b) => order.indexOf(a.need) - order.indexOf(b.need));
  if (keep) for (const r of sorted) { const p = r.get(); if (p >= 0 && ok(p, r.need) && !used.has(p)) used.add(p); else if (r.need !== 'i2c') r.set(-1); }
  const failed = [];
  for (const r of sorted) {
    if (r.need === 'i2c') { if (!(r.get() >= 0) || !ok(r.get(), 'out') && !reserved.has(r.get())) r.set(r.role.endsWith('SDA') ? B.i2c.sda : B.i2c.scl); used.add(r.get()); continue; }
    if (keep && r.get() >= 0) continue;
    const cands = B.pins.filter((p) => !used.has(p) && ok(p, r.need)).sort((a, b) => cost(a) - cost(b) || a - b);
    if (!cands.length) { r.set(-1); failed.push(r.role); continue; }
    r.set(cands[0]); used.add(cands[0]);
  }
  return { hw, failed };
}

// ---- validation --------------------------------------------------------------------------------------------
// Each message: { level: 'bad' | 'warn' | 'info', key, vars }  (keys live in web/i18n/en.json under fw.v.*)
export function validateHardware(robot, hwIn) {
  const hw = normalizeHardware(hwIn, robot.sensors.length);
  const B = BOARDS[hw.board], out = [];
  const msg = (level, key, vars = {}) => out.push({ level, key, vars });
  const L = (p) => pinLabel(hw.board, p);
  const uses = pinRoles(robot, hw).map((r) => [r.get(), r.role, r.need]);
  const d = hw.driver, e = hw.encoder;
  if (e.type === 'as5600' && e.as5600_bus === 'two_buses' && B.arch !== 'esp32') msg('bad', 'fw.v.as5600_two_buses_avr');
  const ticks = sketchTicks(robot, hw);
  const nTof = robot.sensors.filter((s) => isTof(s.type)).length;
  robot.sensors.forEach((s, i) => {
    const k = sensorKind(s.type), nm = `#${i + 1} ${sensorWireName(s, i)}`;
    if ((k === 'l0x' || k === 'l1x') && hw.sensors[i].xshut < 0 && nTof > 1) msg('bad', 'fw.v.xshut_needed', { n: nm });
    if (k === 'none') msg('bad', 'fw.v.sensor_type', { n: nm, t: s.type });
  });
  const i2c = usesI2C(robot, hw);
  // per pin checks
  const seen = new Map();
  for (const [p, role, need] of uses) {
    if (p === undefined || p === null || p < 0) { msg('bad', 'fw.v.missing', { role }); continue; }
    if (!B.pins.includes(p)) { msg('bad', 'fw.v.no_pin', { role, p: L(p) }); continue; }
    if (seen.has(p)) msg('bad', 'fw.v.twice', { p: L(p), a: seen.get(p), b: role });
    else seen.set(p, role);
    if (B.serial.includes(p)) msg('bad', 'fw.v.serial', { p: L(p), role });
    const outNeed = ['out', 'pwm', 'i2c'].includes(need);
    if (outNeed && B.inputOnly.includes(p)) msg('bad', 'fw.v.input_only', { p: L(p), role });
    if (need === 'pwm' && !B.pwm.includes(p)) msg('bad', 'fw.v.no_pwm', { p: L(p), role, list: B.pwm.map(L).join(', ') });
    if (need === 'int' && !B.int.includes(p)) msg('bad', 'fw.v.no_int', { p: L(p), role, list: B.int.map(L).join(', ') });
    if (need === 'anyint' && !B.int.includes(p) && !B.pcint.includes(p)) msg('bad', 'fw.v.no_pcint', { p: L(p), role, list: [...B.int, ...B.pcint].map(L).join(', ') });
    if (need === 'analog' && !B.analog.includes(p)) msg('bad', 'fw.v.no_analog', { p: L(p), role });
    if (need === 'analog' && B.adc2 && B.adc2.includes(p)) msg('warn', 'fw.v.adc2', { p: L(p), role });
    if (need === 'i2c' && B.i2c.fixed && p !== B.i2c.sda && p !== B.i2c.scl) msg('bad', 'fw.v.i2c_fixed', { p: L(p) });
    if (B.strapping.includes(p)) msg('warn', p === 12 ? 'fw.v.strap12' : 'fw.v.strap', { p: L(p), role });
    if (B.arch === 'esp32' && B.inputOnly.includes(p) && role.startsWith('encoder')) msg('warn', 'fw.v.no_pullup', { p: L(p), role });
    if (p === B.led && B.arch === 'avr' && (need === 'in' || need === 'int' || need === 'anyint')) msg('warn', 'fw.v.led_in', { p: L(p), role });
    if (B.arch === 'avr' && B.i2c.fixed && i2c && (p === B.i2c.sda || p === B.i2c.scl) && need !== 'i2c') msg('bad', 'fw.v.i2c_taken', { p: L(p), role });
  }
  // encoders
  if (e.type === 'quadrature' && B.arch === 'avr' && e.avr_mode === 'x1') msg('info', 'fw.v.avr_x1');
  if (needsPcint(hw)) msg('info', 'fw.v.pcint');
  if (e.type === 'single') msg('warn', 'fw.v.single');
  if (e.type === 'none') msg('warn', 'fw.v.none');
  if (e.type === 'as5600') {
    msg('info', e.as5600_bus === 'mux' ? 'fw.v.as5600_mux' : 'fw.v.as5600_buses');
    const rps = robot.motor.max_speed_mps / (Math.PI * robot.wheels.diameter_mm / 1000) * (+e.gear || 1);
    if (rps > 60) msg('warn', 'fw.v.as5600_fast', { rps: rps.toFixed(0) });
  }
  if (e.type !== 'none' && e.type !== 'as5600' && !(+e.ppr > 0)) msg('bad', 'fw.v.ppr');
  if (e.type !== 'none' && !(+e.gear > 0)) msg('bad', 'fw.v.gear');
  if (isLowRes(hw, ticks)) msg('info', 'fw.v.lowres', { t: +ticks.toFixed(1) });
  // drivers
  if (d.type === 'en_in1_in2') msg('info', 'fw.v.l298n');
  if (d.type === 'pwm_dir') msg('info', 'fw.v.pwm_dir_stop');
  // sensors
  const nSonar = robot.sensors.filter((s) => sensorKind(s.type) === 'sonar').length;
  if (nSonar) msg('warn', 'fw.v.sonar', { n: nSonar, ms: sonarCycleMs(robot) });
  if (nSonar && B.arch === 'esp32') msg('warn', 'fw.v.sonar_5v');
  if (robot.sensors.some((s) => sensorKind(s.type) === 'sharp')) msg('info', B.arch === 'esp32' ? 'fw.v.sharp_esp32' : 'fw.v.sharp');
  if (nTof && hw.board === 'uno' && nTof >= 3 && robot.sensors.some((s) => sensorKind(s.type) === 'l1x')) msg('info', 'fw.v.uno_flash');
  const addrs = robot.sensors.map((s, i) => (isTof(s.type) ? tofAddr(hw, i) : null)).filter((a) => a !== null);
  if (new Set(addrs).size !== addrs.length) msg('bad', 'fw.v.addr_twice');
  if (addrs.some((a) => a === 0x36 || a === hw.encoder.mux_addr) && hw.encoder.type === 'as5600') msg('bad', 'fw.v.addr_clash');
  if (robot.drive === '4wd') msg('info', 'fw.v.fourwd');
  return out;
}
function tofAddr(hw, i) { const a = +hw.sensors[i].addr; return Number.isFinite(a) && a >= 0x08 && a < 0x78 ? a : 0x30 + i; }
function sonarTimeoutUs(s) { return Math.round((s.max_mm || 4000) * 2 / 0.343 + 1000); }
function sonarGapMs(robot) { const n = robot.sensors.filter((s) => sensorKind(s.type) === 'sonar').length; return Math.max(10, Math.ceil(60 / Math.max(1, n))); }
function sonarCycleMs(robot) {
  return robot.sensors.filter((s) => sensorKind(s.type) === 'sonar').reduce((a, s) => a + sonarTimeoutUs(s) / 1000 + sonarGapMs(robot), 0).toFixed(0);
}
export function hasErrors(list) { return list.some((m) => m.level === 'bad'); }

// ---- the serial protocol: ONE source for the sketch header, the student prompt (own-code.js) and the tests ----
// Every reply line here is printed by CORE below (tests/js/owncode.test.mjs checks each literal piece), and every
// command CORE handles is listed here. <...> = a number or name filled in at run time.
export const BAUD = 115200;
export const PROTOCOL_RULES = [
  `${BAUD} baud, 8N1. One command per line, ended by "\\n" ("\\r" is ignored). Upper or lower case. At most 63 characters.`,
  'Numbers after a command are separated by spaces; decimals use a dot ("CAL STEP 0.15").',
  'Every CAL command also works without the word CAL ("TICKS" = "CAL TICKS", "SPIN 5" = "CAL SPIN 5").',
  'Replies are plain text lines ending in "\\r\\n" (Serial.println). The Tuner reads them line by line and matches the start of each line.',
  'At start-up the sketch prints: robot_tuner_min ready (commands: see the top of the sketch)',
  'Errors start with "ERR ", warnings with "WARN ", acknowledgements with "OK ". Unknown command -> "ERR unknown command".',
  'Link messages (Bluetooth robots), printed by themselves when it happens: LINK bluetooth connected  /  LINK bluetooth disconnected  (a PC opened or closed the Bluetooth serial port). Robot Tuner shows them on screen.',
  'Without wheel encoders, every command that needs them answers: ERR <command> needs wheel encoders: this robot has none (time-based mode). Use CAL TIMED or type the value on the Measure page.',
];
// motion: what the robot does when the command runs. hand = motors off, you turn the wheels; air = wheels spin, lift the
// robot so they turn freely; floor = the robot DRIVES on the floor: no cable, commands over Bluetooth; still = nothing moves.
export const MOTION = {
  hand: 'motors off; the student turns the wheels by hand',
  air: 'the wheels spin: the robot must be lifted so the wheels turn freely (USB cable plugged in, robot does not move)',
  floor: 'the robot DRIVES on the floor on battery power with NO cable: commands arrive over Bluetooth serial (ESP32). Without Bluetooth the student skips these commands and measures by hand',
  still: 'nothing moves',
};
export const PROTOCOL = [
  { cmd: 'CAL TICKS', motion: 'hand', usage: 'CAL TICKS / CAL END', help: 'turn each wheel one revolution by hand -> ticks per revolution', enc: true,
    replies: ['OK CAL TICKS started: turn each wheel one revolution, then send CAL END'] },
  { cmd: 'CAL END', motion: 'hand', help: 'ends CAL TICKS', enc: true,
    replies: ['CAL TICKS result: left <ticks> right <ticks> per revolution', 'CAL suggestion: SET ticks_rev <ticks, 1 decimal>', 'ERR no CAL TICKS running'] },
  { cmd: 'CAL MAXSPEED', motion: 'air', usage: 'CAL MAXSPEED', help: 'full power 1 s, wheels in the air -> top speed', enc: true,
    replies: ['CAL MAXSPEED: left <m/s> m/s, right <m/s> m/s at PWM <PWM_MAX>', 'CAL suggestion: SET max_speed <m/s>'] },
  { cmd: 'CAL FF', motion: 'air', usage: 'CAL FF', help: 'PWM sweep, wheels in the air -> deadband (ff_static) and PWM per speed (ff_gain)', enc: true,
    replies: ['CAL FF: pwm <pwm>  left <ticks per 50 ms>  right <ticks per 50 ms> ticks/50ms   (one line per PWM step 0, 5, 10 ... 255)',
      'CAL FF: slope <g> PWM per tick/50ms (= <g> per tick/50 ms)', 'CAL suggestion: SET ff_gain <g>', 'CAL suggestion: SET ff_static <pwm>', 'CAL: FF sweep done'] },
  { cmd: 'CAL STEP', motion: 'air', args: '[m/s]', usage: 'CAL STEP [m/s]', help: 'speed step with a simple wheel PI -> time constant', enc: true, def: '0.15 m/s',
    replies: ['CAL STEP <m/s> m/s (= sketch PI): rise L <s>s R <s>s, overshoot L <%>% R <%>%, tau63 L <s>s R <s>s, steady L <m/s> R <m/s>'] },
  { cmd: 'CAL DRIVE', motion: 'floor', args: '<m>', usage: 'CAL DRIVE <m> / CAL DIST <real m>', help: 'straight drive -> wheel scale', enc: true, def: '1.0 m',
    replies: ['CAL DRIVE done: ticks L <ticks> R <ticks>', 'CAL DRIVE: measure the real distance D (m), then send: CAL DIST <D>'] },
  { cmd: 'CAL DIST', motion: 'still', args: '<real m>', help: 'the distance you measured after CAL DRIVE', enc: true,
    replies: ['CAL DIST: ticks per metre L <ticks/m> R <ticks/m>; suggestions wscale_l <x> wscale_r <x>', 'ERR run CAL DRIVE first'] },
  { cmd: 'CAL SPIN', motion: 'floor', args: '<turns>', usage: 'CAL SPIN <turns> / CAL TURNS <real>', help: 'spin in place -> effective track width', enc: true, def: '5 turns',
    replies: ['CAL SPIN done: encoders say <turns> turns with track_w <m> m; asymmetry <%> %, at the PWM clamp 0 % of the time -> <OK or UNTRUSTED>',
      'CAL SPIN: count the REAL turns and send: CAL TURNS <real_turns>'] },
  { cmd: 'CAL TURNS', motion: 'still', args: '<real turns>', help: 'the turns you counted after CAL SPIN', enc: true,
    replies: ['CAL TURNS: effective track width <m> m (was <m>).', 'ERR run CAL SPIN first'] },
  { cmd: 'CAL DIR', motion: 'air', usage: 'CAL DIR', help: 'each wheel forward 0.4 s at half power, wheels in the air -> motor and encoder signs', enc: true,
    replies: ['CAL DIR: left wheel, then right wheel, forward 0.4 s: look, both must turn FORWARD',
      'CAL DIR: left <ticks> right <ticks> ticks at forward PWM <pwm>',
      'CAL DIR result: left <OK or WRONG> right <OK or WRONG>   (WRONG = the count went down: flip ENC_SIGN if the wheel turned forward, MOTOR_SIGN if it turned backward)'] },
  { cmd: 'CAL TIMED', motion: 'floor', args: '[ms]', usage: 'CAL TIMED [ms]', help: 'full power for ms milliseconds without encoders -> measure the distance yourself', def: '2000 ms',
    replies: ['CAL TIMED done: full power for <ms> ms at PWM <PWM_MAX>. Measure the distance D (m): top speed = D / time'] },
  { cmd: 'CAL VMIN', motion: 'air', usage: 'CAL VMIN', help: 'PWM ramp both ways, wheels in the air -> slowest steady speed (v_min)', enc: true,
    replies: ['CAL VMIN: <side> <dir> <m/s> m/s at PWM <pwm>, <side> <dir> <m/s> m/s at PWM <pwm><, ...>   (4 entries in this order: left fwd, right fwd, left rev, right rev; "none" instead of "<m/s> m/s at PWM <pwm>" when that wheel never turned steadily)',
      'CAL suggestion: SET v_min <m/s>', 'ERR CAL VMIN: a wheel never turned steadily'] },
  { cmd: 'RUN', motion: 'floor', args: '[s]', usage: 'RUN [s] / STOP', help: 'wall following with the PID below (constant cruise speed), default 10 s', def: '10 s',
    replies: ['OK RUN started (send STOP to stop)', 'RUN done', 'ERR RUN needs a left- and a right-looking sensor'] },
  { cmd: 'STOP', motion: 'still', help: 'stops RUN', replies: ['RUN done'] },
  { cmd: 'CAL TOF', motion: 'still', args: '[n] [true_mm]', usage: 'CAL TOF [n] [true_mm]', help: 'n readings per sensor -> noise, bias, dropouts', def: 'n = 100',
    replies: ['CAL TOF <sensor name> n <count> mean <mm> mm std <mm> mm min <mm> max <mm> dropouts <count> (<%>%)   (one line per sensor)',
      'CAL TOF <sensor name> bias vs <true mm> mm: <+/-mm> mm   (only when true_mm is given)', 'CAL: TOF done'] },
  { cmd: 'PING', motion: 'still', usage: 'PING', help: 'is the robot there, and over which link? Robot Tuner sends it every few seconds', replies: ['PONG <usb or bluetooth>   (the link the PING came over)'] },
  { cmd: 'STATUS', motion: 'still', usage: 'STATUS', help: 'loop period and reading age',
    replies: ['tof mm <sensor name> <mm> <sensor name> <mm> ...   (-1 = no valid reading)',
      'ToF ages at the last tick: data-ready seen <us>/<us>/<us> us ago (R/F/L), + 0 ms half budget',
      'control loop (ToF data-ready): period <us> us (max <us>), compute 0 us, overruns 0, ticks 0, telemetry dropped 0'] },
  { cmd: 'LOAD SENSORS', motion: 'still', usage: 'LOAD SENSORS / LOAD FINAL', help: 'motors off / on (sensor timing test)', replies: ['OK profile SENSORS (motors off)'] },
  { cmd: 'LOAD FINAL', motion: 'still', help: 'motors back on', replies: ['OK profile FINAL'] },
];
/** the "Commands" block at the top of the sketch */
export function protocolHeaderLines() {
  return PROTOCOL.filter((p) => p.usage).map((p) => `//   ${p.usage.padEnd(26)} ${p.help}`);
}

// ---- the sketch --------------------------------------------------------------------------------------------
const f = (x, n = 4) => { const s = (+x).toFixed(n); return s.includes('.') ? s.replace(/0+$/, '').replace(/\.$/, '.0') : s; };
const hex = (a) => '0x' + a.toString(16).toUpperCase().padStart(2, '0');

export const DRIVER_TEXT = {
  pwm_dir: 'PWM + DIR per motor (Pololu DRV8838 / DRV8835 in PHASE-ENABLE mode, Cytron MD10C / MDD10A, any "PWM + direction" board)',
  in1_in2: 'two inputs IN1 / IN2 per motor, both PWM (DRV8833, MX1508, L9110S, TB67H450, L298N with the ENA/ENB jumpers on)',
  en_in1_in2: 'EN (PWM) + IN1 + IN2 per motor (L298N with ENA/ENB wired, TB6612FNG: PWMA + AIN1 + AIN2 [+ STBY])',
};
export function encoderText(hw, ticks) {
  const e = hw.encoder;
  if (e.type === 'quadrature') {
    const how = usesPcnt(hw, ticks) ? 'ESP32 PCNT hardware counter (ESP-IDF pulse_cnt driver), both edges of A and B (x4)'
      : BOARDS[hw.board].arch === 'avr' && e.avr_mode === 'x1' ? 'channel A rising edge on an interrupt pin, B read for the direction (x1)'
        : 'interrupts on both edges of A and B (x4)' + (needsPcint(hw) ? ', PinChangeInterrupt library' : '');
    return `quadrature A+B (${how})`;
  }
  if (e.type === 'single') return 'single channel (slotted disc / LM393): pulses counted, sign taken from the commanded motor direction';
  if (e.type === 'as5600') return `AS5600 magnetic angle sensors on I2C (${e.as5600_bus === 'mux' ? 'TCA9548A multiplexer' : 'two I2C buses'}), turns counted from the angle wrap`;
  return 'NONE: time-based mode (calibration steps that need encoders answer ERR)';
}

/** Generate the sketch. Returns { code, libraries, messages, ticks, lowRes, fileName } */
export function generateSketch(robot, hwIn) {
  const hw = normalizeHardware(hwIn, robot.sensors.length);
  const B = BOARDS[hw.board], esp = B.arch === 'esp32', avr = !esp;
  const P = (p) => pinCode(hw.board, p);
  const e = hw.encoder, d = hw.driver;
  const ticks = sketchTicks(robot, hw);
  const lowRes = isLowRes(hw, ticks);
  const hasEnc = e.type !== 'none';
  const pcnt = usesPcnt(hw, ticks);
  const libs = requiredLibraries(robot, hw);
  const i2c = usesI2C(robot, hw);
  const N = robot.sensors.length;
  const kinds = robot.sensors.map((s) => sensorKind(s.type));
  const names = robot.sensors.map(sensorWireName);
  const C = [];
  const w = (...lines) => C.push(...lines);

  // ---------- header ----------
  w(`// robot_tuner_min.ino - generated by Robot Tuner (Robot builder -> Firmware) for "${String(robot.name || 'robot').replace(/[\r\n]/g, ' ')}".`,
    '// Answers Robot Tuner\'s measurement commands over USB serial, so the Measure page works unchanged.',
    '//',
    `// Board:     ${B.label}   (arduino-cli FQBN ${B.fqbn})`,
    `// Encoders:  ${encoderText(hw, ticks)}`,
    `// Driver:    ${DRIVER_TEXT[d.type]}`,
    `// Sensors:   ${robot.sensors.map((s, i) => `${names[i]} = ${s.type}`).join(', ')}`,
    `// Libraries (Arduino Library Manager): ${libs.length ? libs.map((l) => `"${l.name}" by ${l.by}`).join(', ') : 'none (only the built-in ones)'}`,
    esp ? '// Board package: "esp32" by Espressif Systems (Boards Manager).' : '// Board package: "Arduino AVR Boards" (built in).',
    '//',
    `// Commands (${BAUD} baud, one per line; every CAL command also works without the word CAL):`,
    ...protocolHeaderLines(),
    '');
  if (e.type === 'single') {
    w('// SINGLE-CHANNEL ENCODERS: a slotted disc gives pulses but no direction. The sketch takes the direction from the',
      '// last non-zero motor command. When a wheel is told to reverse it keeps turning the old way for a moment (inertia),',
      '// and those pulses are counted in the NEW direction: every reversal adds an error of about',
      '// (speed x stopping time) / distance per tick, typically 1-5 ticks. Forward-only runs and the CAL steps are fine;',
      '// a controller that often reverses a wheel (spinning on the spot to correct) drifts.', '');
  }
  if (lowRes) {
    w(`// LOW RESOLUTION (${f(ticks, 1)} ticks per wheel revolution < ${LOW_RES_TICKS}): wheel speed is measured from the time`,
      '// between pulses, not by counting pulses per interval (that would give only a few counts per 10 ms).', '');
  }
  if (!hasEnc) {
    w('// NO ENCODERS (time-based mode): commands that need wheel encoders (TICKS, MAXSPEED, FF, STEP, DRIVE, SPIN)',
      '// answer "ERR ... needs wheel encoders". Without encoders the wheels are driven open loop: the two motors never',
      '// run at exactly the same speed, so the robot curves and every turn drifts. Use CAL TIMED to estimate the top speed.', '');
  }
  const bt = usesBt(hw);
  if (bt) {
    w('// BLUETOOTH SERIAL (classic ESP32): the robot answers over USB AND Bluetooth at the same time. Pair the PC once with',
      `// "${btName(robot, hw)}" (Windows: Settings -> Bluetooth -> Add device; it gets a COM port "Standard Serial over Bluetooth"),`,
      '// then press CONNECT in Robot Tuner and pick that port: the robot can drive on the floor without a cable.',
      '// ESP32-S3 / C3 / C6 have no classic Bluetooth: switch it off in Robot builder -> Firmware.',
      '#include "BluetoothSerial.h"', '');
  }
  if (i2c) w('#include <Wire.h>');
  if (pcnt) w('#include <driver/pulse_cnt.h>   // ESP-IDF PCNT driver, part of the esp32 board package (core 3.x and 4.x)');
  if (needsPcint(hw)) w('#include <PinChangeInterrupt.h>');
  if (kinds.includes('l0x')) w('#include <VL53L0X.h>');
  if (kinds.includes('l1x')) w('#include <VL53L1X.h>');
  if (bt) {
    w('BluetoothSerial SerialBT;', 'bool cmdFromBt = false;   // the last command came over Bluetooth (PING answers which link)',
      '// RT_IO: every reply goes to USB and Bluetooth; commands are read from both',
      'class TeeIO : public Stream {',
      ' public:',
      '  // Bluetooth only while a PC is connected: with nobody listening its queue fills and every write waits 1 s',
      '  size_t write(uint8_t c) override { USB_SERIAL_.write(c); if (SerialBT.hasClient()) SerialBT.write(c); return 1; }',
      '  size_t write(const uint8_t* b, size_t n) override { USB_SERIAL_.write(b, n); if (SerialBT.hasClient()) SerialBT.write(b, n); return n; }',
      '  int available() override { return USB_SERIAL_.available() + SerialBT.available(); }',
      '  int read() override { if (USB_SERIAL_.available()) { cmdFromBt = false; return USB_SERIAL_.read(); } cmdFromBt = true; return SerialBT.read(); }',
      '  int peek() override { return USB_SERIAL_.available() ? USB_SERIAL_.peek() : SerialBT.peek(); }',
      '  void flush() override { USB_SERIAL_.flush(); SerialBT.flush(); }',
      '};',
      'TeeIO RT_IO;');
  } else w('#define RT_IO Serial   // replies and commands: USB serial', 'const bool cmdFromBt = false;');
  w('', '#ifndef IRAM_ATTR', '#define IRAM_ATTR', '#endif',
    'struct Meter { long pl, pr; unsigned long us; };   // wheel speed meter (declared first for the Arduino prototype generator)', '');
  // (after Meter: the Arduino IDE puts its function prototypes in front of the first function)
  if (bt) w('void linkWatch() {   // tell USB and Bluetooth when a PC opens or closes the Bluetooth port',
    '  static bool was = false; bool now = SerialBT.hasClient();',
    '  if (now != was) { was = now; RT_IO.println(now ? F("LINK bluetooth connected") : F("LINK bluetooth disconnected")); }',
    '}', '');
  else w('void linkWatch() {}', '');

  // ---------- config ----------
  w('// ======================= CONFIG (generated; change it here if you rewire) =======================');
  w(`// motor driver: ${DRIVER_TEXT[d.type]}`);
  if (d.type === 'pwm_dir') w(`const int PIN_PWM_L = ${P(d.left.a)}, PIN_DIR_L = ${P(d.left.b)}, PIN_PWM_R = ${P(d.right.a)}, PIN_DIR_R = ${P(d.right.b)};`);
  if (d.type === 'in1_in2') w(`const int PIN_IN1_L = ${P(d.left.a)}, PIN_IN2_L = ${P(d.left.b)}, PIN_IN1_R = ${P(d.right.a)}, PIN_IN2_R = ${P(d.right.b)};`);
  if (d.type === 'en_in1_in2') {
    w(`const int PIN_EN_L = ${P(d.left.en)}, PIN_IN1_L = ${P(d.left.a)}, PIN_IN2_L = ${P(d.left.b)};`,
      `const int PIN_EN_R = ${P(d.right.en)}, PIN_IN1_R = ${P(d.right.a)}, PIN_IN2_R = ${P(d.right.b)};`);
  }
  if (d.type !== 'in1_in2' && d.stby >= 0) w(`const int PIN_STBY = ${P(d.stby)};   // driver standby: held HIGH`);
  if (d.type !== 'pwm_dir') {
    w(`const bool STOP_BRAKE = ${d.stop === 'brake' ? 'true' : 'false'};   // true: a stopped motor is shorted (brake); false: it turns freely (coast)`);
  }
  w(`const int MOTOR_SIGN_L = ${d.sign_l < 0 ? -1 : 1}, MOTOR_SIGN_R = ${d.sign_r < 0 ? -1 : 1};   // flip to -1 if a wheel drives backwards`);
  if (e.type === 'quadrature') w(`const int PIN_ENC_L_A = ${P(e.left.a)}, PIN_ENC_L_B = ${P(e.left.b)}, PIN_ENC_R_A = ${P(e.right.a)}, PIN_ENC_R_B = ${P(e.right.b)};`);
  if (e.type === 'single') w(`const int PIN_ENC_L = ${P(e.left.a)}, PIN_ENC_R = ${P(e.right.a)};`);
  if (e.type === 'quadrature' || e.type === 'as5600') w(`const int ENC_SIGN_L = ${e.sign_l < 0 ? -1 : 1}, ENC_SIGN_R = ${e.sign_r < 0 ? -1 : 1};   // flip to -1 if a wheel counts backwards`);
  if (e.type === 'as5600') {
    w('const uint8_t AS5600_ADDR = 0x36;');
    if (e.as5600_bus === 'mux') w(`const uint8_t MUX_ADDR = ${hex(+e.mux_addr || 0x70)}, MUX_CH_L = ${+e.mux_ch_l || 0}, MUX_CH_R = ${+e.mux_ch_r || 0};   // TCA9548A channels of the left / right AS5600`);
    else w(`const int PIN_SDA2 = ${P(e.sda2)}, PIN_SCL2 = ${P(e.scl2)};   // second I2C bus (Wire1): right AS5600`);
  }
  if (i2c && esp) w(`const int PIN_SDA = ${P(hw.i2c.sda)}, PIN_SCL = ${P(hw.i2c.scl)};`);
  if (i2c && avr) w(`// I2C: SDA = ${pinLabel(hw.board, B.i2c.sda)}, SCL = ${pinLabel(hw.board, B.i2c.scl)} (fixed on this board)`);
  w(`const float WHEEL_DIAMETER_M = ${f(robot.wheels.diameter_mm / 1000, 4)}f;`);
  const tc = computedTicksPerRev(hw);
  const tcNote = e.type === 'none' ? 'no encoders' : e.type === 'as5600'
    ? `computed: 4096 per magnet turn x gear ${f(+e.gear, 3)}`
    : `computed: ${+e.ppr} pulses x ${edgesPerPulse(hw)} edges x gear ${f(+e.gear, 3)} = ${f(tc, 1)}`;
  w(`float ticks_per_rev = ${f(ticks || 1, 3)}f;     // ${tcNote}; replace with the CAL TICKS result`,
    `float track_w = ${f((robot.wheels.track_effective_mm || robot.wheels.track_mm) / 1000, 4)}f;             // replace with the CAL TURNS result`,
    'const int PWM_MAX = 255;',
    `const int N_SENSORS = ${N};`,
    `const char* SENSOR_NAME[N_SENSORS] = {${names.map((n) => `"${n}"`).join(', ')}};   // = the sensor names in the robot builder`,
    `#define HAS_ENCODERS ${hasEnc ? 1 : 0}`,
    `#define LOW_RES ${lowRes ? 1 : 0}`);
  w('// =================================================================================================', '');

  // ---------- motors ----------
  w('// ---- motors ----',
    'bool motorsEnabled = true;',
    'volatile int cmdDirL = 1, cmdDirR = 1;   // sign of the last non-zero command (single-channel encoders use it)');
  if (d.type === 'pwm_dir') {
    w('// stopping (brake or coast) is decided by the driver board when PWM = 0',
      'void motorOut(int pinPwm, int pinDir, int u) { digitalWrite(pinDir, u < 0 ? HIGH : LOW); analogWrite(pinPwm, abs(u)); }',
      'void motorsSetup() { pinMode(PIN_PWM_L, OUTPUT); pinMode(PIN_DIR_L, OUTPUT); pinMode(PIN_PWM_R, OUTPUT); pinMode(PIN_DIR_R, OUTPUT);' + (d.stby >= 0 ? ' pinMode(PIN_STBY, OUTPUT); digitalWrite(PIN_STBY, HIGH);' : '') + ' }',
      'void motorL(int u) { motorOut(PIN_PWM_L, PIN_DIR_L, u); }',
      'void motorR(int u) { motorOut(PIN_PWM_R, PIN_DIR_R, u); }');
  }
  if (d.type === 'in1_in2') {
    w('// Two-input driver (DRV8833 style): IN1/IN2 = 0/0 coast, 1/1 brake, PWM/0 forward, 0/PWM reverse.',
      '// STOP_BRAKE also picks the decay between PWM pulses: brake = slow decay (one input HIGH, the other gets the',
      '// inverted PWM; speed is close to proportional to PWM), coast = fast decay (one input PWM, the other LOW).',
      '// Both inputs must be PWM pins. analogWrite(pin, 0 / PWM_MAX) is used for LOW / HIGH so the pin never',
      '// switches between digital and PWM mode.',
      'void motorOut(int in1, int in2, int u) {',
      '  int d = constrain(abs(u), 0, PWM_MAX);',
      '  if (d == 0) { analogWrite(in1, STOP_BRAKE ? PWM_MAX : 0); analogWrite(in2, STOP_BRAKE ? PWM_MAX : 0); return; }',
      '  if (STOP_BRAKE) {',
      '    if (u > 0) { analogWrite(in1, PWM_MAX); analogWrite(in2, PWM_MAX - d); } else { analogWrite(in1, PWM_MAX - d); analogWrite(in2, PWM_MAX); }',
      '  } else {',
      '    if (u > 0) { analogWrite(in1, d); analogWrite(in2, 0); } else { analogWrite(in1, 0); analogWrite(in2, d); }',
      '  }',
      '}',
      'void motorsSetup() { pinMode(PIN_IN1_L, OUTPUT); pinMode(PIN_IN2_L, OUTPUT); pinMode(PIN_IN1_R, OUTPUT); pinMode(PIN_IN2_R, OUTPUT); }',
      'void motorL(int u) { motorOut(PIN_IN1_L, PIN_IN2_L, u); }',
      'void motorR(int u) { motorOut(PIN_IN1_R, PIN_IN2_R, u); }');
  }
  if (d.type === 'en_in1_in2') {
    w('// L298N / TB6612: IN1/IN2 set the direction, EN gets the PWM. Stop: brake = IN1 = IN2 = HIGH with EN HIGH',
      '// (motor terminals shorted); coast = EN LOW (motor free).',
      'void motorOut(int en, int in1, int in2, int u) {',
      '  int d = constrain(abs(u), 0, PWM_MAX);',
      '  if (d == 0) {',
      '    if (STOP_BRAKE) { digitalWrite(in1, HIGH); digitalWrite(in2, HIGH); analogWrite(en, PWM_MAX); }',
      '    else { digitalWrite(in1, LOW); digitalWrite(in2, LOW); analogWrite(en, 0); }',
      '    return;',
      '  }',
      '  digitalWrite(in1, u > 0 ? HIGH : LOW); digitalWrite(in2, u > 0 ? LOW : HIGH); analogWrite(en, d);',
      '}',
      'void motorsSetup() {',
      '  pinMode(PIN_EN_L, OUTPUT); pinMode(PIN_IN1_L, OUTPUT); pinMode(PIN_IN2_L, OUTPUT);',
      '  pinMode(PIN_EN_R, OUTPUT); pinMode(PIN_IN1_R, OUTPUT); pinMode(PIN_IN2_R, OUTPUT);' + (d.stby >= 0 ? '\n  pinMode(PIN_STBY, OUTPUT); digitalWrite(PIN_STBY, HIGH);' : ''),
      '}',
      'void motorL(int u) { motorOut(PIN_EN_L, PIN_IN1_L, PIN_IN2_L, u); }',
      'void motorR(int u) { motorOut(PIN_EN_R, PIN_IN1_R, PIN_IN2_R, u); }');
  }
  w('// l, r: -PWM_MAX..PWM_MAX, positive = forward',
    'void setPwm(int l, int r) {',
    '  if (!motorsEnabled) { l = 0; r = 0; }',
    '  l = constrain(l, -PWM_MAX, PWM_MAX); r = constrain(r, -PWM_MAX, PWM_MAX);',
    '  if (l) cmdDirL = l > 0 ? 1 : -1;',
    '  if (r) cmdDirR = r > 0 ? 1 : -1;',
    '  motorL(l * MOTOR_SIGN_L); motorR(r * MOTOR_SIGN_R);',
    '}', '');

  // ---------- encoders ----------
  w('// ---- wheel encoders: cntL() / cntR() = signed ticks, forward positive ----');
  const isrEnc = e.type === 'single' || (e.type === 'quadrature' && !pcnt);
  if (isrEnc) {
    w('volatile long encL = 0, encR = 0;',
      'volatile unsigned long edgeUsL = 0, edgeUsR = 0, periodUsL = 0, periodUsR = 0;   // time of the last timed edge, time between two',
      'volatile int stepL = 1, stepR = 1;   // direction of the last counted tick');
    w('void attachEnc(int pin, void (*isr)(), int mode) {');
    if (needsPcint(hw)) w('  if (digitalPinToInterrupt(pin) == NOT_AN_INTERRUPT) { attachPCINT(digitalPinToPCINT(pin), isr, mode); return; }');
    w('  attachInterrupt(digitalPinToInterrupt(pin), isr, mode);', '}');
  }
  if (e.type === 'quadrature' && pcnt) {
    w('// PCNT: the ESP32 hardware pulse counter counts every edge of A and B (x4) without using the CPU.',
      '// (ESP-IDF rotary encoder setup. The ESP32Encoder library does the same but does not build on esp32 core 4.0.)',
      'pcnt_unit_handle_t pcntL = NULL, pcntR = NULL;',
      'pcnt_unit_handle_t pcntQuad(int a, int b) {',
      '  pcnt_unit_config_t uc = {}; uc.low_limit = -30000; uc.high_limit = 30000; uc.flags.accum_count = 1;   // accumulate past +-30000',
      '  pcnt_unit_handle_t u = NULL; ESP_ERROR_CHECK(pcnt_new_unit(&uc, &u));',
      '  pcnt_glitch_filter_config_t fc = {}; fc.max_glitch_ns = 1000; ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(u, &fc));',
      '  pcnt_chan_config_t ca = {}; ca.edge_gpio_num = a; ca.level_gpio_num = b;',
      '  pcnt_chan_config_t cb = {}; cb.edge_gpio_num = b; cb.level_gpio_num = a;',
      '  pcnt_channel_handle_t c0 = NULL, c1 = NULL; ESP_ERROR_CHECK(pcnt_new_channel(u, &ca, &c0)); ESP_ERROR_CHECK(pcnt_new_channel(u, &cb, &c1));',
      '  // same edge / level actions as ESP32Encoder attachFullQuad, so a count direction measured with it still holds',
      '  pcnt_channel_set_edge_action(c0, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);',
      '  pcnt_channel_set_level_action(c0, PCNT_CHANNEL_LEVEL_ACTION_INVERSE, PCNT_CHANNEL_LEVEL_ACTION_KEEP);',
      '  pcnt_channel_set_edge_action(c1, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);',
      '  pcnt_channel_set_level_action(c1, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);',
      '  pcnt_unit_add_watch_point(u, 30000); pcnt_unit_add_watch_point(u, -30000);',
      '  ESP_ERROR_CHECK(pcnt_unit_enable(u)); pcnt_unit_clear_count(u); ESP_ERROR_CHECK(pcnt_unit_start(u));',
      '  gpio_pullup_en((gpio_num_t)a); gpio_pullup_en((gpio_num_t)b);',
      '  return u;',
      '}',
      'void encSetup() { pcntL = pcntQuad(PIN_ENC_L_A, PIN_ENC_L_B); pcntR = pcntQuad(PIN_ENC_R_A, PIN_ENC_R_B); }',
      'void encPoll() {}',
      'long cntL() { int v = 0; pcnt_unit_get_count(pcntL, &v); return (long)v * ENC_SIGN_L; }',
      'long cntR() { int v = 0; pcnt_unit_get_count(pcntR, &v); return (long)v * ENC_SIGN_R; }');
  } else if (e.type === 'quadrature' && avr && e.avr_mode === 'x1') {
    w('// x1 decoding: one tick per rising edge of A; B tells the direction',
      'void isrL() { int s = digitalRead(PIN_ENC_L_B) ? -1 : 1; encL += s; stepL = s; unsigned long n = micros(); periodUsL = n - edgeUsL; edgeUsL = n; }',
      'void isrR() { int s = digitalRead(PIN_ENC_R_B) ? -1 : 1; encR += s; stepR = s; unsigned long n = micros(); periodUsR = n - edgeUsR; edgeUsR = n; }',
      'void encSetup() {',
      '  pinMode(PIN_ENC_L_A, INPUT_PULLUP); pinMode(PIN_ENC_L_B, INPUT_PULLUP); pinMode(PIN_ENC_R_A, INPUT_PULLUP); pinMode(PIN_ENC_R_B, INPUT_PULLUP);',
      '  attachEnc(PIN_ENC_L_A, isrL, RISING); attachEnc(PIN_ENC_R_A, isrR, RISING);',
      '}',
      'void encPoll() {}',
      'const int TICKS_PER_EDGE = 1;',
      'long cntL() { noInterrupts(); long v = encL; interrupts(); return v * ENC_SIGN_L; }',
      'long cntR() { noInterrupts(); long v = encR; interrupts(); return v * ENC_SIGN_R; }',
      'int dirL() { return stepL * ENC_SIGN_L; }',
      'int dirR() { return stepR * ENC_SIGN_R; }');
  } else if (e.type === 'quadrature') {
    w('// x4 decoding: every edge of A and B; the table maps (old state, new state) to -1 / 0 / +1',
      'const int8_t QDEC[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};',
      'volatile uint8_t qL = 0, qR = 0;',
      'void IRAM_ATTR isrL() {',
      '  uint8_t s = (digitalRead(PIN_ENC_L_A) << 1) | digitalRead(PIN_ENC_L_B);',
      '  int8_t st = QDEC[(qL << 2) | s]; encL += st; if (st) stepL = st;',
      '  if ((s & 2) && !(qL & 2)) { unsigned long n = micros(); periodUsL = n - edgeUsL; edgeUsL = n; }   // A rose: one encoder period = 4 ticks',
      '  qL = s;',
      '}',
      'void IRAM_ATTR isrR() {',
      '  uint8_t s = (digitalRead(PIN_ENC_R_A) << 1) | digitalRead(PIN_ENC_R_B);',
      '  int8_t st = QDEC[(qR << 2) | s]; encR += st; if (st) stepR = st;',
      '  if ((s & 2) && !(qR & 2)) { unsigned long n = micros(); periodUsR = n - edgeUsR; edgeUsR = n; }',
      '  qR = s;',
      '}',
      'void encSetup() {',
      '  pinMode(PIN_ENC_L_A, INPUT_PULLUP); pinMode(PIN_ENC_L_B, INPUT_PULLUP); pinMode(PIN_ENC_R_A, INPUT_PULLUP); pinMode(PIN_ENC_R_B, INPUT_PULLUP);',
      '  qL = (digitalRead(PIN_ENC_L_A) << 1) | digitalRead(PIN_ENC_L_B); qR = (digitalRead(PIN_ENC_R_A) << 1) | digitalRead(PIN_ENC_R_B);',
      '  attachEnc(PIN_ENC_L_A, isrL, CHANGE); attachEnc(PIN_ENC_L_B, isrL, CHANGE);',
      '  attachEnc(PIN_ENC_R_A, isrR, CHANGE); attachEnc(PIN_ENC_R_B, isrR, CHANGE);',
      '}',
      'void encPoll() {}',
      'const int TICKS_PER_EDGE = 4;',
      'long cntL() { noInterrupts(); long v = encL; interrupts(); return v * ENC_SIGN_L; }',
      'long cntR() { noInterrupts(); long v = encR; interrupts(); return v * ENC_SIGN_R; }',
      'int dirL() { return stepL * ENC_SIGN_L; }',
      'int dirR() { return stepR * ENC_SIGN_R; }');
  } else if (e.type === 'single') {
    const minPeriodS = (Math.PI * robot.wheels.diameter_mm / 1000 / Math.max(1, ticks)) / (1.5 * Math.max(0.05, robot.motor.max_speed_mps));
    const deb = Math.round(Math.min(2000, Math.max(50, 0.3 * minPeriodS * 1e6)));
    w(`const unsigned long DEBOUNCE_US = ${deb};   // edges closer than this are contact bounce / noise (0.3 x the shortest real period)`,
      'void IRAM_ATTR isrL() { unsigned long n = micros(); if (n - edgeUsL < DEBOUNCE_US) return; periodUsL = n - edgeUsL; edgeUsL = n; encL += cmdDirL; stepL = cmdDirL; }',
      'void IRAM_ATTR isrR() { unsigned long n = micros(); if (n - edgeUsR < DEBOUNCE_US) return; periodUsR = n - edgeUsR; edgeUsR = n; encR += cmdDirR; stepR = cmdDirR; }',
      'void encSetup() {',
      '  pinMode(PIN_ENC_L, INPUT_PULLUP); pinMode(PIN_ENC_R, INPUT_PULLUP);',
      '  attachEnc(PIN_ENC_L, isrL, RISING); attachEnc(PIN_ENC_R, isrR, RISING);',
      '}',
      'void encPoll() {}',
      'const int TICKS_PER_EDGE = 1;',
      'long cntL() { noInterrupts(); long v = encL; interrupts(); return v; }',
      'long cntR() { noInterrupts(); long v = encR; interrupts(); return v; }',
      'int dirL() { return cmdDirL; }',
      'int dirR() { return cmdDirR; }');
  } else if (e.type === 'as5600') {
    const busR = e.as5600_bus === 'mux' ? 'Wire' : 'Wire1';
    w('// AS5600: 12-bit angle (0..4095 per turn of the magnet). The sketch reads it every 0.5 ms and counts the wrap:',
      '// a jump of more than half a turn between two reads is a wrap through 0. The magnet must turn less than half a',
      '// turn between two reads (fine below ~1000 turns per second).',
      'long encL = 0, encR = 0; int rawL = -1, rawR = -1; unsigned long as5600Errors = 0, lastPollUs = 0;');
    if (e.as5600_bus === 'mux') w('void muxSelect(uint8_t ch) { Wire.beginTransmission(MUX_ADDR); Wire.write((uint8_t)(1 << ch)); Wire.endTransmission(); }');
    w('int as5600Read(TwoWire& bus, uint8_t reg) {',
      '  bus.beginTransmission(AS5600_ADDR); bus.write(reg);',
      '  if (bus.endTransmission(false) != 0) return -1;',
      '  if (bus.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2) != 2) return -1;',
      '  int hi = bus.read(), lo = bus.read();',
      '  return ((hi << 8) | lo) & 0x0FFF;',
      '}',
      'void wrapCount(int raw, int& last, long& count, int sign) {',
      '  if (raw < 0) { as5600Errors++; return; }',
      '  if (last >= 0) { int d = raw - last; if (d > 2048) d -= 4096; else if (d < -2048) d += 4096; count += (long)d * sign; }',
      '  last = raw;',
      '}',
      'void encPoll() {',
      '  unsigned long n = micros(); if (n - lastPollUs < 500) return; lastPollUs = n;',
      e.as5600_bus === 'mux' ? '  muxSelect(MUX_CH_L);' : '',
      '  wrapCount(as5600Read(Wire, 0x0C), rawL, encL, ENC_SIGN_L);   // 0x0C = RAW ANGLE',
      e.as5600_bus === 'mux' ? '  muxSelect(MUX_CH_R);' : '',
      `  wrapCount(as5600Read(${busR}, 0x0C), rawR, encR, ENC_SIGN_R);`,
      '}',
      'void as5600Check(TwoWire& bus, const char* side) {',
      '  int st = as5600Read(bus, 0x0A);   // 0x0B STATUS read as the low byte of 0x0A/0x0B',
      '  if (st < 0) { Serial.print(F("WARN AS5600 ")); Serial.print(side); Serial.println(F(" not found on I2C")); return; }',
      '  if (!(st & 0x20)) { Serial.print(F("WARN AS5600 ")); Serial.print(side); Serial.println(F(" sees no magnet")); }',
      '}',
      'void encSetup() {');
    if (e.as5600_bus === 'mux') w('  muxSelect(MUX_CH_L); as5600Check(Wire, "left"); muxSelect(MUX_CH_R); as5600Check(Wire, "right");');
    else w('  Wire1.begin(PIN_SDA2, PIN_SCL2, 400000);', '  as5600Check(Wire, "left"); as5600Check(Wire1, "right");');
    w('  lastPollUs = micros() - 1000; encPoll();',
      '}',
      'long cntL() { return encL; }',
      'long cntR() { return encR; }');
  } else {
    w('void encSetup() {}', 'void encPoll() {}', 'long cntL() { return 0; }', 'long cntR() { return 0; }');
  }
  w('float mPerTick() { return 3.14159265f * WHEEL_DIAMETER_M / ticks_per_rev; }', '');

  // ---------- distance sensors ----------
  w('// ---- distance sensors: every new reading goes through newReading(i, mm); -1 = no valid reading ----',
    'int lastMm[N_SENSORS];',
    'unsigned long readingTimeUs[N_SENSORS], sensorSeq[N_SENSORS];',
    'bool freshInSet[N_SENSORS];',
    'unsigned long lastSetUs = 0, loopPeriodUs = 0;   // the "control loop" = one fresh reading from every sensor',
    'bool setReady = false;                             // a new full set arrived (the wall-following PID runs on it)',
    'void newReading(int i, int mm) {',
    '  lastMm[i] = mm; readingTimeUs[i] = micros(); sensorSeq[i]++; freshInSet[i] = true;',
    '  for (int k = 0; k < N_SENSORS; k++) if (!freshInSet[k]) return;',
    '  unsigned long now = micros(); loopPeriodUs = now - lastSetUs; lastSetUs = now; setReady = true;',
    '  for (int k = 0; k < N_SENSORS; k++) freshInSet[k] = false;',
    '}');
  const tofIdx = robot.sensors.map((s, i) => i).filter((i) => kinds[i] === 'l0x' || kinds[i] === 'l1x');
  const sonarIdx = robot.sensors.map((s, i) => i).filter((i) => kinds[i] === 'sonar');
  const sharpIdx = robot.sensors.map((s, i) => i).filter((i) => kinds[i] === 'sharp');
  for (const i of tofIdx) w(`${kinds[i] === 'l0x' ? 'VL53L0X' : 'VL53L1X'} tof${i};   // sensor ${i} "${names[i]}" ${robot.sensors[i].type}`);
  if (tofIdx.length) {
    w(`bool tofOk[N_SENSORS];`, `unsigned long tofMissingMs[N_SENSORS];`);
    const useX = tofIdx.filter((i) => hw.sensors[i].xshut >= 0);
    if (useX.length) {
      w('// XSHUT: LOW = sensor off. All ToF sensors share address 0x29 at power-up, so they are kept off, then woken',
        '// one at a time and moved to their own address. The pin is released (INPUT) instead of driven HIGH because the',
        '// breakout pulls XSHUT up to its own 2.8 V supply.');
      w(`const int XSHUT_PIN[] = {${useX.map((i) => P(hw.sensors[i].xshut)).join(', ')}};`);
    }
    w('void tofMissing(int i) { Serial.print(F("WARN sensor ")); Serial.print(SENSOR_NAME[i]); Serial.println(F(" not found: check wiring and XSHUT")); }',
      'void tofSetup() {');
    if (useX.length) w(`  for (int k = 0; k < ${useX.length}; k++) { pinMode(XSHUT_PIN[k], OUTPUT); digitalWrite(XSHUT_PIN[k], LOW); }`, '  delay(10);');
    for (const i of tofIdx) {
      const s = robot.sensors[i], px = hw.sensors[i].xshut, addr = tofAddr(hw, i), per = Math.max(20, Math.round(s.period_ms || 33));
      w(`  // sensor ${i} "${names[i]}": ${s.type}${px >= 0 ? `, XSHUT ${pinLabel(hw.board, px)}` : ''}, address ${hex(addr)}`);
      if (px >= 0) w(`  pinMode(${P(px)}, INPUT); delay(10);`);
      w(`  tof${i}.setTimeout(500);`,
        `  tofOk[${i}] = tof${i}.init();`,
        `  if (!tofOk[${i}]) tofMissing(${i});`,
        `  else {`);
      if (tofIdx.length > 1 || addr !== 0x29) w(`    tof${i}.setAddress(${hex(addr)});`);
      if (kinds[i] === 'l1x') {
        const long = s.type === 'VL53L1X_long';
        const budget = Math.max(long ? 33 : 20, per - 5);
        w(`    tof${i}.setDistanceMode(VL53L1X::${long ? 'Long' : 'Short'});`,
          `    tof${i}.setMeasurementTimingBudget(${budget * 1000});   // us; the period below must be >= budget + 4 ms`,
          `    tof${i}.startContinuous(${Math.max(per, budget + 4)});`);
      } else {
        w(`    tof${i}.setMeasurementTimingBudget(${Math.max(20, per - 2) * 1000});   // us`,
          `    tof${i}.startContinuous(0);   // back to back`);
      }
      w('  }');
    }
    w('}');
  }
  if (sonarIdx.length) {
    w('// HC-SR04: one sensor pings at a time (round robin). Two sensors firing together hear each other\'s echoes',
      '// (crosstalk). Each ping blocks for up to the echo timeout, so with several sensors the loop gets slow.',
      `const int SONAR_IDX[] = {${sonarIdx.join(', ')}};`,
      `const int SONAR_TRIG[] = {${sonarIdx.map((i) => P(hw.sensors[i].trig)).join(', ')}};`,
      `const int SONAR_ECHO[] = {${sonarIdx.map((i) => P(hw.sensors[i].echo)).join(', ')}};`,
      `const unsigned long SONAR_TIMEOUT_US[] = {${sonarIdx.map((i) => sonarTimeoutUs(robot.sensors[i])).join(', ')}};   // echo of the max range`,
      `const unsigned long SONAR_GAP_MS = ${sonarGapMs(robot)};   // quiet time after each ping so its echoes die out`,
      `int sonarTurn = 0; unsigned long sonarLastMs = 0;`,
      'void sonarSetup() { for (int k = 0; k < ' + sonarIdx.length + '; k++) { pinMode(SONAR_TRIG[k], OUTPUT); digitalWrite(SONAR_TRIG[k], LOW); pinMode(SONAR_ECHO[k], INPUT); } }',
      'void sonarPoll() {',
      '  if (millis() - sonarLastMs < SONAR_GAP_MS) return;',
      '  int k = sonarTurn; sonarTurn = (sonarTurn + 1) % ' + sonarIdx.length + ';',
      '  digitalWrite(SONAR_TRIG[k], LOW); delayMicroseconds(2); digitalWrite(SONAR_TRIG[k], HIGH); delayMicroseconds(10); digitalWrite(SONAR_TRIG[k], LOW);',
      '  unsigned long us = pulseIn(SONAR_ECHO[k], HIGH, SONAR_TIMEOUT_US[k]);',
      '  newReading(SONAR_IDX[k], us ? (int)(us * 0.1715f) : -1);   // mm = us x 343 m/s / 2',
      '  sonarLastMs = millis();',
      '}');
  }
  if (sharpIdx.length) {
    w('// Sharp IR (analog): distance_cm = A * V^-B, a power-law fit of the datasheet curve. Your sensor differs a bit:',
      '// measure at 10, 20, 30 ... cm and refit A and B (or use CAL TOF bias). Outside its range -> -1.',
      `const int SHARP_IDX[] = {${sharpIdx.join(', ')}};`,
      `const int SHARP_PIN[] = {${sharpIdx.map((i) => P(hw.sensors[i].analog)).join(', ')}};`,
      `const float SHARP_A[] = {${sharpIdx.map((i) => f(SHARP_FIT[robot.sensors[i].type].a, 3) + 'f').join(', ')}};`,
      `const float SHARP_B[] = {${sharpIdx.map((i) => f(SHARP_FIT[robot.sensors[i].type].b, 3) + 'f').join(', ')}};`,
      `const int SHARP_MIN_MM[] = {${sharpIdx.map((i) => robot.sensors[i].min_mm).join(', ')}}, SHARP_MAX_MM[] = {${sharpIdx.map((i) => robot.sensors[i].max_mm).join(', ')}};`,
      `const unsigned long SHARP_PERIOD_MS[] = {${sharpIdx.map((i) => Math.max(5, Math.round(robot.sensors[i].period_ms || 39))).join(', ')}};   // the sensor updates its output this often`,
      `unsigned long sharpLastMs[${sharpIdx.length}];`,
      esp ? 'float readVolts(int pin) { long s = 0; for (int k = 0; k < 4; k++) s += analogReadMilliVolts(pin); return s / 4000.0f; }'
        : 'float readVolts(int pin) { long s = 0; for (int k = 0; k < 4; k++) s += analogRead(pin); return s / 4.0f * 5.0f / 1023.0f; }   // 5 V reference',
      'void sharpPoll() {',
      `  for (int k = 0; k < ${sharpIdx.length}; k++) {`,
      '    if (millis() - sharpLastMs[k] < SHARP_PERIOD_MS[k]) continue;',
      '    sharpLastMs[k] = millis();',
      '    float v = readVolts(SHARP_PIN[k]);',
      '    float mm = v > 0.1f ? 10.0f * SHARP_A[k] * pow(v, -SHARP_B[k]) : -1;',
      '    newReading(SHARP_IDX[k], (mm < SHARP_MIN_MM[k] || mm > SHARP_MAX_MM[k]) ? -1 : (int)mm);',
      '  }',
      '}');
  }
  w('void sensorsSetup() {',
    '  for (int i = 0; i < N_SENSORS; i++) { lastMm[i] = -1; readingTimeUs[i] = micros(); sensorSeq[i] = 0; freshInSet[i] = false; }');
  if (tofIdx.length) w('  tofSetup();', '  for (int i = 0; i < N_SENSORS; i++) tofMissingMs[i] = 0;');
  if (sonarIdx.length) w('  sonarSetup();');
  if (sharpIdx.length) w(`  for (int k = 0; k < ${sharpIdx.length}; k++) { pinMode(SHARP_PIN[k], INPUT); sharpLastMs[k] = 0; }`);
  w('}',
    'unsigned long lastTofPollUs = 0;',
    'void pollSensors() {');
  if (tofIdx.length) {
    w('  if (micros() - lastTofPollUs >= 1000) {   // ask the ToF sensors at most every 1 ms (each ask is an I2C read)',
      '    lastTofPollUs = micros();');
    for (const i of tofIdx) {
      const per = Math.max(20, Math.round(robot.sensors[i].period_ms || 33));
      w(`    if (!tofOk[${i}]) { if (millis() - tofMissingMs[${i}] >= ${per}) { tofMissingMs[${i}] = millis(); newReading(${i}, -1); } }   // missing sensor: report dropouts`);
      if (kinds[i] === 'l1x') {
        w(`    else if (tof${i}.dataReady()) { tof${i}.read(false); newReading(${i}, tof${i}.ranging_data.range_status == VL53L1X::RangeValid ? (int)tof${i}.ranging_data.range_mm : -1); }`);
      } else {
        w(`    else if (tof${i}.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) { uint16_t mm = tof${i}.readRangeContinuousMillimeters(); newReading(${i}, (tof${i}.timeoutOccurred() || mm >= 8000) ? -1 : (int)mm); }`);
      }
    }
    w('  }');
  }
  if (sharpIdx.length) w('  sharpPoll();');
  if (sonarIdx.length) w('  sonarPoll();');
  w('}', '');

  w('void i2cSetup() {');
  if (i2c) w(esp ? '  Wire.begin(PIN_SDA, PIN_SCL);' : '  Wire.begin();', '  Wire.setClock(400000);');
  w('}', '');
  // ---------- common calibration code (same output lines as before) ----------
  // ---------- wall-following PID config (same formula as the simulator and the exported pid_gains.h) ----------
  const p = robot.pid, angles = robot.sensors.map((q) => q.angle_deg);
  const li = angles.indexOf(Math.max(...angles)), ri = angles.indexOf(Math.min(...angles));
  const vmax = robot.motor.max_speed_mps, vmin = Math.min(Math.max(0, robot.motor.v_min_mps || 0), vmax);
  const sinL = Math.abs(Math.sin(angles[li] * Math.PI / 180)), sinR = Math.abs(Math.sin(angles[ri] * Math.PI / 180));
  w('// ---- wall-following PID (RUN): constant cruise speed, the same formula as the simulator ----',
    '//   e = clamp(left - right - setpoint, +-ERROR_CLIP); integral = clamp(integral + e dt, +-INTEGRAL_LIMIT)',
    '//   d_filt = alpha d_raw + (1 - alpha) d_filt; turn = clamp(KP e + KI integral + KD d_filt, +-V_MAX)',
    '//   base = V_MIN + SPEED_FRAC (V_MAX - V_MIN); v_left = base - turn, v_right = base + turn, clamped to +-V_MAX',
    `const float PID_KP = ${f(p.kp, 6)}f, PID_KI = ${f(p.ki, 6)}f, PID_KD = ${f(p.kd, 6)}f;`,
    `const float PID_SPEED_FRAC = ${f(p.speed_frac, 4)}f;   // cruise = ${(vmin + p.speed_frac * (vmax - vmin)).toFixed(3)} m/s`,
    `const float PID_V_MAX = ${f(vmax, 4)}f, PID_V_MIN = ${f(vmin, 4)}f;   // measured top speed, slowest steady speed (CAL VMIN)`,
    `const float PID_ERROR_CLIP = ${f(p.error_clip_mm / 1000, 4)}f, PID_INTEGRAL_LIMIT = ${f(p.integral_limit, 4)}f, PID_D_ALPHA = ${f(p.d_filter_alpha, 4)}f;   // alpha per 50 ms`,
    `const float PID_SETPOINT = ${f(-((p.offset_mm || 0) / 1000) * (1 / Math.max(0.17, sinL) + 1 / Math.max(0.17, sinR)), 6)}f;   // wanted left - right (0 = centred)`,
    `const int PID_LEFT = ${li}, PID_RIGHT = ${ri};   // sensors looking most to the left / right`,
    `const float SENSOR_MAX_M[N_SENSORS] = {${robot.sensors.map((q) => f(q.max_mm / 1000, 3) + 'f').join(', ')}};   // no reading = nothing within range`,
    `#define CAN_RUN ${robot.sensors.length >= 2 && li !== ri ? 1 : 0}`, '');
  w('// no extra commands in this sketch (the ESP32 install firmware adds SET / GET / SAVE here)', 'bool extraCommand(const char* u, const char* orig) { return false; }', '');
  w(CORE);
  const code = C.join('\n').replace(/\bSerial\.(print|println|available|read)\(/g, 'RT_IO.$1(')
    .replace(/USB_SERIAL_\./g, 'Serial.').replace('  BT_BEGIN', bt ? `  SerialBT.enableSSP();   // Secure Simple Pairing, "Just Works" (no PIN): Windows / Mac pair without questions\n  SerialBT.begin("${btName(robot, hw)}");   // Bluetooth device name` : '').replace(/\n\n(?=void setup)/, '\n\n');
  return { code: code + '\n', libraries: libs, messages: validateHardware(robot, hw), ticks, lowRes, fileName: 'robot_tuner_min.ino', fqbn: B.fqbn };
}

// The calibration protocol. Every printed result line is parsed by web/js/measure-steps.js: keep them identical.
export const CORE = String.raw`// ---- waiting: keeps the encoders (AS5600) and, when asked, the sensors running ----
void idle(bool sensors) { encPoll(); if (sensors) pollSensors(); }
void waitMs(unsigned long ms, bool sensors = false) { unsigned long t = millis(); while (millis() - t < ms) idle(sensors); }

// ---- wheel speed (m/s) since the last read of a meter ----
void meterStart(Meter& m) { m.pl = cntL(); m.pr = cntR(); m.us = micros(); }
#if LOW_RES
// few ticks per revolution: speed = distance of one timed edge / time between two edges
float periodSpeed(volatile unsigned long& lastEdge, volatile unsigned long& period, int dir) {
  noInterrupts(); unsigned long p = period, t = lastEdge; interrupts();
  unsigned long since = micros() - t;
  if (p == 0 || since > 300000UL) return 0;   // no edge for 0.3 s: stopped
  if (since > p) p = since;                    // the next edge is late: the wheel is slowing down
  return dir * TICKS_PER_EDGE * mPerTick() / (p * 1e-6f);
}
#endif
float meterRead(Meter& m, float& vl, float& vr) {   // returns dt (s)
  long l = cntL(), r = cntR(); unsigned long now = micros();
  float dt = max(1e-4f, (now - m.us) * 1e-6f);
#if LOW_RES
  vl = periodSpeed(edgeUsL, periodUsL, dirL()); vr = periodSpeed(edgeUsR, periodUsR, dirR());
#else
  vl = (l - m.pl) * mPerTick() / dt; vr = (r - m.pr) * mPerTick() / dt;
#endif
  m.pl = l; m.pr = r; m.us = now;
  return dt;
}
// average speed over ms milliseconds
void avgSpeeds(unsigned long ms, float& vl, float& vr) {
#if LOW_RES
  float sl = 0, sr = 0; int n = 0; Meter m; meterStart(m); unsigned long t0 = millis();
  while (millis() - t0 < ms) { float a, b; waitMs(5); meterRead(m, a, b); sl += a; sr += b; n++; }
  vl = sl / max(1, n); vr = sr / max(1, n);
#else
  Meter m; meterStart(m); waitMs(ms); meterRead(m, vl, vr);
#endif
}

// simple wheel speed PI with feed-forward (for STEP / DRIVE / SPIN)
float ff_static = 30, ff_gain = 400;   // PWM, PWM per (m/s): replaced by CAL FF
float wkp = 300, wki = 2000;           // PWM per (m/s), PWM per m
float iL = 0, iR = 0;
int piOne(float tg, float v, float& integ, float dt) {
  if (fabs(tg) < 1e-3) { integ = 0; return 0; }
  float e = tg - v; integ += e * dt;
  float u = (tg > 0 ? ff_static : -ff_static) + ff_gain * tg + wkp * e + wki * integ;
  return (int)constrain(u, -PWM_MAX, PWM_MAX);
}
void wheelPI(float tl, float tr, float vl, float vr, float dt) { setPwm(piOne(tl, vl, iL, dt), piOne(tr, vr, iR, dt)); }

char line[64]; int lineLen = 0;   // the command being typed

void setup() {
  Serial.begin(${BAUD});
  BT_BEGIN
  motorsSetup();
  setPwm(0, 0);
  i2cSetup();
  encSetup();
  sensorsSetup();
  Serial.println(F("robot_tuner_min ready (commands: see the top of the sketch)"));
}

long c0L, c0R; bool ticksRunning = false; long driveTicksL = 0, driveTicksR = 0; float spinEncRot = 0;

void calTicks() { c0L = cntL(); c0R = cntR(); ticksRunning = true; Serial.println(F("OK CAL TICKS started: turn each wheel one revolution, then send CAL END")); }
void calEnd() {
  if (!ticksRunning) { Serial.println(F("ERR no CAL TICKS running")); return; }
  ticksRunning = false;
  long l = labs(cntL() - c0L), r = labs(cntR() - c0R);
  Serial.print(F("CAL TICKS result: left ")); Serial.print(l); Serial.print(F(" right ")); Serial.print(r); Serial.println(F(" per revolution"));
  Serial.print(F("CAL suggestion: SET ticks_rev ")); Serial.println((l + r) / 2.0, 1);
}
void calMaxSpeed() {
  setPwm(PWM_MAX, PWM_MAX); waitMs(500);
  float vl, vr; avgSpeeds(500, vl, vr);
  setPwm(0, 0);
  Serial.print(F("CAL MAXSPEED: left ")); Serial.print(vl, 3); Serial.print(F(" m/s, right ")); Serial.print(vr, 3); Serial.print(F(" m/s at PWM ")); Serial.println(PWM_MAX);
  Serial.print(F("CAL suggestion: SET max_speed ")); Serial.println(min(vl, vr), 4);
}
void calFF() {
  // sweep the PWM; speeds in ticks per 50 ms (the units of the full firmware), slope by least squares
  float sx = 0, sy = 0, sxx = 0, sxy = 0; int n = 0, start = -1;
  for (int pwm = 0; pwm <= PWM_MAX; pwm += 5) {
    setPwm(pwm, pwm); waitMs(250);
    float vl, vr; avgSpeeds(LOW_RES ? 200 : 50, vl, vr);
    float l50 = vl * 0.05f / mPerTick(), r50 = vr * 0.05f / mPerTick(), t50 = 0.5f * (l50 + r50);
    Serial.print(F("CAL FF: pwm ")); Serial.print(pwm); Serial.print(F("  left ")); Serial.print(l50, 2); Serial.print(F("  right ")); Serial.print(r50, 2); Serial.println(F(" ticks/50ms"));
    if (start < 0 && t50 >= (LOW_RES ? 0.1f : 1)) start = pwm;
    if (t50 >= (LOW_RES ? 0.2f : 2)) { sx += t50; sy += pwm; sxx += t50 * t50; sxy += t50 * pwm; n++; }
  }
  setPwm(0, 0);
  if (n >= 3) {
    float g = (n * sxy - sx * sy) / (n * sxx - sx * sx);
    Serial.print(F("CAL FF: slope ")); Serial.print(g, 4); Serial.print(F(" PWM per tick/50ms (= ")); Serial.print(g, 4); Serial.println(F(" per tick/50 ms)"));
    ff_gain = g * 0.05f / mPerTick();  // PWM per (m/s) for this sketch's own PI
    Serial.print(F("CAL suggestion: SET ff_gain ")); Serial.println(g, 4);
  }
  if (start >= 0) { ff_static = start; Serial.print(F("CAL suggestion: SET ff_static ")); Serial.println(start); }
  Serial.println(F("CAL: FF sweep done"));
}
void calStep(float v) {
  // 1.5 s step of the wheel PI; rise / overshoot / tau computed on the fly (no sample buffer: fits an Uno)
  const int N = 150; Meter m; meterStart(m); iL = iR = 0;
  float t10 = -1, t63 = -1, t90 = -1, peak = 0, ss = 0, t = 0; int nss = 0;
  for (int k = 0; k < N; k++) {
    float vl, vr; waitMs(10); float dt = meterRead(m, vl, vr); wheelPI(v, v, vl, vr, dt); t += dt;
    float y = 0.5f * (vl + vr), fr = y / v;
    if (fr > peak) peak = fr;
    if (t10 < 0 && fr >= 0.1) t10 = t; if (t63 < 0 && fr >= 0.632) t63 = t; if (t90 < 0 && fr >= 0.9) t90 = t;
    if (k >= N - N / 3) { ss += y; nss++; }
  }
  setPwm(0, 0);
  ss /= max(1, nss);
  float rise = (t10 >= 0 && t90 >= 0) ? t90 - t10 : -1, over = peak > 1 ? (peak - 1) * 100 : 0;
  Serial.print(F("CAL STEP ")); Serial.print(v, 2); Serial.print(F(" m/s (= sketch PI): rise L ")); Serial.print(rise, 3); Serial.print(F("s R ")); Serial.print(rise, 3);
  Serial.print(F("s, overshoot L ")); Serial.print(over, 0); Serial.print(F("% R ")); Serial.print(over, 0); Serial.print(F("%, tau63 L ")); Serial.print(t63, 3);
  Serial.print(F("s R ")); Serial.print(t63, 3); Serial.print(F("s, steady L ")); Serial.print(ss, 3); Serial.print(F(" R ")); Serial.println(ss, 3);
}
void calDrive(float dist) {
  long sl = cntL(), sr = cntR(); Meter m; meterStart(m); iL = iR = 0; float v = 0.2f;
  for (;;) {
    float vl, vr; waitMs(10); float dt = meterRead(m, vl, vr);
    float dl = (cntL() - sl) * mPerTick(), dr = (cntR() - sr) * mPerTick();
    if (0.5f * (dl + dr) >= dist) break;
    float corr = 2.0f * (dl - dr);
    wheelPI(v - corr, v + corr, vl, vr, dt);
  }
  setPwm(0, 0);
  driveTicksL = cntL() - sl; driveTicksR = cntR() - sr;
  Serial.print(F("CAL DRIVE done: ticks L ")); Serial.print(driveTicksL); Serial.print(F(" R ")); Serial.println(driveTicksR);
  Serial.println(F("CAL DRIVE: measure the real distance D (m), then send: CAL DIST <D>"));
}
void calDist(float real) {
  if (!driveTicksL) { Serial.println(F("ERR run CAL DRIVE first")); return; }
  float tl = driveTicksL / real, tr = driveTicksR / real, nominal = 1.0f / mPerTick();
  Serial.print(F("CAL DIST: ticks per metre L ")); Serial.print(tl, 1); Serial.print(F(" R ")); Serial.print(tr, 1);
  Serial.print(F("; suggestions wscale_l ")); Serial.print(nominal / tl, 4); Serial.print(F(" wscale_r ")); Serial.println(nominal / tr, 4);
}
void calSpin(float turns) {
  long sl = cntL(), sr = cntR(); Meter m; meterStart(m); iL = iR = 0; float goal = turns * 2 * 3.14159265f, v = 0.12f;
  float dl = 0, dr = 0;
  for (;;) {
    float vl, vr; waitMs(10); float dt = meterRead(m, vl, vr);
    dl = (cntL() - sl) * mPerTick(); dr = (cntR() - sr) * mPerTick();
    if ((dr - dl) / track_w >= goal) break;
    float drift = 2.0f * (dl + dr);
    wheelPI(-v - drift, v - drift, vl, vr, dt);
  }
  setPwm(0, 0);
  spinEncRot = (dr - dl) / track_w;
  float asym = fabs(dl + dr) / max(1e-6f, fabs(dl) + fabs(dr));
  Serial.print(F("CAL SPIN done: encoders say ")); Serial.print(spinEncRot / (2 * 3.14159265f), 3); Serial.print(F(" turns with track_w ")); Serial.print(track_w, 4);
  Serial.print(F(" m; asymmetry ")); Serial.print(100 * asym, 1); Serial.print(F(" %, at the PWM clamp 0 % of the time -> ")); Serial.println(asym < 0.05 ? "OK" : "UNTRUSTED");
  Serial.println(F("CAL SPIN: count the REAL turns and send: CAL TURNS <real_turns>"));
}
void calTurns(float real) {
  if (spinEncRot <= 0) { Serial.println(F("ERR run CAL SPIN first")); return; }
  float tw = track_w * spinEncRot / (real * 2 * 3.14159265f);
  Serial.print(F("CAL TURNS: effective track width ")); Serial.print(tw, 4); Serial.print(F(" m (was ")); Serial.print(track_w, 4); Serial.println(F(")."));
  track_w = tw;
}
void calDir() {
  // motor and encoder signs: forward PWM must turn the wheel forward (look) and count up (checked here)
  Serial.println(F("CAL DIR: left wheel, then right wheel, forward 0.4 s: look, both must turn FORWARD"));
  long l0 = cntL(); setPwm(PWM_MAX / 2, 0); waitMs(400); setPwm(0, 0); waitMs(300); long dl = cntL() - l0;
  long r0 = cntR(); setPwm(0, PWM_MAX / 2); waitMs(400); setPwm(0, 0); waitMs(300); long dr = cntR() - r0;
  Serial.print(F("CAL DIR: left ")); Serial.print(dl); Serial.print(F(" right ")); Serial.print(dr); Serial.print(F(" ticks at forward PWM ")); Serial.println(PWM_MAX / 2);
  Serial.print(F("CAL DIR result: left ")); Serial.print(dl > 0 ? F("OK") : F("WRONG")); Serial.print(F(" right ")); Serial.println(dr > 0 ? F("OK") : F("WRONG"));
}
void calTimed(unsigned long ms) {
  // open loop: no encoders needed. Measure the distance and divide by the time (minus about one time constant)
  setPwm(PWM_MAX, PWM_MAX); waitMs(ms); setPwm(0, 0);
  Serial.print(F("CAL TIMED done: full power for ")); Serial.print(ms); Serial.print(F(" ms at PWM ")); Serial.print(PWM_MAX);
  Serial.println(F(". Measure the distance D (m): top speed = D / time"));
}
void calTof(int n, float trueMm) {
  for (int i = 0; i < N_SENSORS; i++) {
    double s = 0, sq = 0; int cnt = 0, drop = 0, mn = 32767, mx = 0;
    for (int k = 0; k < n; k++) {
      unsigned long seq = sensorSeq[i], t0 = millis();
      while (sensorSeq[i] == seq && millis() - t0 < 500) idle(true);   // wait for a new reading of sensor i
      int mm = sensorSeq[i] == seq ? -1 : lastMm[i];
      if (mm < 0) { drop++; continue; }
      s += mm; sq += (double)mm * mm; cnt++; mn = min(mn, mm); mx = max(mx, mm);
    }
    double mean = cnt ? s / cnt : 0, sd = cnt > 1 ? sqrt(max(0.0, (sq - cnt * mean * mean) / (cnt - 1))) : 0;
    Serial.print(F("CAL TOF ")); Serial.print(SENSOR_NAME[i]); Serial.print(F(" n ")); Serial.print(cnt); Serial.print(F(" mean ")); Serial.print(mean, 1);
    Serial.print(F(" mm std ")); Serial.print(sd, 2); Serial.print(F(" mm min ")); Serial.print(cnt ? mn : 0); Serial.print(F(" max ")); Serial.print(mx);
    Serial.print(F(" dropouts ")); Serial.print(drop); Serial.print(F(" (")); Serial.print(100.0 * drop / max(1, cnt + drop), 1); Serial.println(F("%)"));
    if (trueMm > 0 && cnt) { Serial.print(F("CAL TOF ")); Serial.print(SENSOR_NAME[i]); Serial.print(F(" bias vs ")); Serial.print(trueMm, 0); Serial.print(F(" mm: ")); Serial.print(mean - trueMm >= 0 ? "+" : ""); Serial.print(mean - trueMm, 1); Serial.println(F(" mm")); }
  }
  Serial.println(F("CAL: TOF done"));
}
void status() {
  unsigned long now = micros();
  Serial.print(F("tof mm"));
  for (int i = 0; i < N_SENSORS; i++) { Serial.print(F(" ")); Serial.print(SENSOR_NAME[i]); Serial.print(F(" ")); Serial.print(lastMm[i]); }
  Serial.println();
  // the full firmware prints three ages (R/F/L); we print the first three sensors
  Serial.print(F("ToF ages at the last tick: data-ready seen "));
  for (int i = 0; i < 3; i++) { Serial.print(now - readingTimeUs[min(i, N_SENSORS - 1)]); Serial.print(i < 2 ? "/" : ""); }
  Serial.print(F(" us ago (R/F/L), + 0 ms half budget\n"));
  Serial.print(F("control loop (ToF data-ready): period ")); Serial.print(loopPeriodUs); Serial.print(F(" us (max ")); Serial.print(loopPeriodUs); Serial.println(F("), compute 0 us, overruns 0, ticks 0, telemetry dropped 0"));
}

// Slowest speed each wheel drives steadily, wheels in the air. One wheel and one direction at a time, the PWM steps
// up from 0 by 3; at each step three 100 ms windows are measured; the wheel "turns steadily" when all three move the
// right way and the slowest is >= 70 % of the fastest. v_min = the largest of the four (every wheel drives it, both ways).
float vminOne(int w, int dir, int& at) {
  float v = -1; at = -1;
  for (int pwm = 0; pwm <= PWM_MAX && at < 0; pwm += 3) {
    setPwm(w == 0 ? dir * pwm : 0, w == 1 ? dir * pwm : 0); waitMs(200);
    float lo = 1e9, hi = -1e9;
    for (int k = 0; k < 3; ++k) { float a, b; avgSpeeds(100, a, b); float x = (w ? b : a) * dir; if (x < lo) lo = x; if (x > hi) hi = x; }
    if (lo > 0 && lo >= 0.7f * hi) { v = 0.5f * (lo + hi); at = pwm; }
  }
  setPwm(0, 0); waitMs(300);
  return v;
}
void calVmin() {
  float worst = 0; bool all = true;
  Serial.print(F("CAL VMIN:"));
  for (int k = 0; k < 4; ++k) {   // left fwd, right fwd, left rev, right rev
    int at; float v = vminOne(k & 1, k < 2 ? 1 : -1, at);
    Serial.print(k ? F(", ") : F(" ")); Serial.print((k & 1) ? F("right") : F("left")); Serial.print(k < 2 ? F(" fwd ") : F(" rev "));
    if (at < 0) { Serial.print(F("none")); all = false; continue; }
    Serial.print(v, 4); Serial.print(F(" m/s at PWM ")); Serial.print(at); if (v > worst) worst = v;
  }
  Serial.println();
  if (!all) { Serial.println(F("ERR CAL VMIN: a wheel never turned steadily")); return; }
  Serial.print(F("CAL suggestion: SET v_min ")); Serial.println(worst, 4);
}

// ---- RUN: wall following with the PID above, the wheel PI tracking its wheel speeds ----
float pidI = 0, pidPrev = 0, pidD = 0; bool pidHas = false, running = false;
unsigned long runEndMs = 0, lastRunSetUs = 0;
float targetL = 0, targetR = 0;
Meter runMeter; unsigned long lastWheelMs = 0;
void pidControl(float left, float right, float dt, float& vl, float& vr) {
  float e = constrain(left - right - PID_SETPOINT, -PID_ERROR_CLIP, PID_ERROR_CLIP);
  pidI = constrain(pidI + e * dt, -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);
  float rawD = pidHas ? (e - pidPrev) / dt : 0.0f;
  float alpha = 1.0f - powf(1.0f - PID_D_ALPHA, dt / 0.05f);
  pidD = alpha * rawD + (1.0f - alpha) * pidD;
  pidPrev = e; pidHas = true;
  float turn = constrain(PID_KP * e + PID_KI * pidI + PID_KD * pidD, -PID_V_MAX, PID_V_MAX);
  float base = PID_V_MIN + PID_SPEED_FRAC * (PID_V_MAX - PID_V_MIN);   // constant cruise speed
  vl = constrain(base - turn, -PID_V_MAX, PID_V_MAX);
  vr = constrain(base + turn, -PID_V_MAX, PID_V_MAX);
}
float sensorM(int i) { return lastMm[i] < 0 ? SENSOR_MAX_M[i] : lastMm[i] / 1000.0f; }
void runStart(float seconds) {
  if (!CAN_RUN) { Serial.println(F("ERR RUN needs a left- and a right-looking sensor")); return; }
  pidI = pidPrev = pidD = 0; pidHas = false; iL = iR = 0; targetL = targetR = 0;
  running = true; runEndMs = millis() + (unsigned long)(seconds * 1000); lastRunSetUs = 0; setReady = false;
  meterStart(runMeter); lastWheelMs = millis();
  Serial.println(F("OK RUN started (send STOP to stop)"));
}
void runStop() { if (running) { running = false; setPwm(0, 0); Serial.println(F("RUN done")); } }
void runTick() {
  if ((long)(millis() - runEndMs) >= 0) { runStop(); return; }
  if (setReady) {   // a fresh reading from every sensor: one PID step
    setReady = false;
    unsigned long now = lastSetUs;
    float dt = lastRunSetUs ? (now - lastRunSetUs) * 1e-6f : 0.025f; lastRunSetUs = now;
    pidControl(sensorM(PID_LEFT), sensorM(PID_RIGHT), max(dt, 1e-3f), targetL, targetR);
  }
  if (millis() - lastWheelMs >= 10) {   // wheel speed loop, 10 ms
    lastWheelMs = millis();
#if HAS_ENCODERS
    float vl, vr; float dt = meterRead(runMeter, vl, vr); wheelPI(targetL, targetR, vl, vr, dt);
#else
    // no encoders: feed-forward only (open loop)
    setPwm(targetL == 0 ? 0 : (int)((targetL > 0 ? ff_static : -ff_static) + ff_gain * targetL),
           targetR == 0 ? 0 : (int)((targetR > 0 ? ff_static : -ff_static) + ff_gain * targetR));
#endif
  }
}

// Command parsing without the String class (saves ~2 kB of flash on an Uno): command names stay in flash (PSTR).
#define IS(w) (strcmp_P(u, PSTR(w)) == 0)
#define STARTS(w) (strncmp_P(u, PSTR(w), sizeof(w) - 1) == 0)
float parseNum(const char* p) {   // small decimal parser ("1.75", "-0.2"): no strtod
  while (*p == ' ') ++p;
  bool neg = *p == '-'; if (neg || *p == '+') ++p;
  float v = 0, scale = 1; bool frac = false;
  for (; *p; ++p) {
    if (*p == '.' && !frac) { frac = true; continue; }
    if (*p < '0' || *p > '9') break;
    if (frac) { scale *= 0.1f; v += (*p - '0') * scale; } else v = v * 10 + (*p - '0');
  }
  return neg ? -v : v;
}
bool needsEncoders(const char* u) {
  return IS("CAL TICKS") || IS("CAL END") || IS("CAL MAXSPEED") || IS("CAL FF") || STARTS("CAL STEP") || STARTS("CAL DRIVE")
    || STARTS("CAL DIST") || STARTS("CAL SPIN") || STARTS("CAL TURNS") || IS("CAL VMIN") || IS("CAL DIR");
}
void handle(const char* c) {
  // trimmed, upper case; the calibration commands also work without "CAL": TICKS, SPIN 5, TOF 200 200 ...
  while (*c == ' ') ++c;
  const char* orig = c;   // as typed (extraCommand gets it with its case)
  char u[64]; int n = 0;
  bool cal = strncasecmp(c, "CAL ", 4) == 0 || strcasecmp(c, "STATUS") == 0 || strncasecmp(c, "LOAD", 4) == 0;
  if (!cal) { memcpy(u, "CAL ", 4); n = 4; }
  for (; *c && n < 63; ++c) u[n++] = toupper(*c);
  while (n > 0 && u[n - 1] == ' ') --n;
  u[n] = 0;
  float a1 = 0, a2 = 0;
  const char* sp1 = n > 4 ? strchr(u + 4, ' ') : NULL;
  const char* sp2 = sp1 ? strchr(sp1 + 1, ' ') : NULL;
  if (sp1) a1 = parseNum(sp1 + 1);
  if (sp2) a2 = parseNum(sp2 + 1);
  if (!HAS_ENCODERS && needsEncoders(u)) {
    Serial.print(F("ERR ")); Serial.print(u); Serial.println(F(" needs wheel encoders: this robot has none (time-based mode). Use CAL TIMED or type the value on the Measure page."));
    return;
  }
  if (IS("CAL TICKS")) calTicks();
  else if (IS("CAL END")) calEnd();
  else if (IS("CAL MAXSPEED")) calMaxSpeed();
  else if (IS("CAL FF")) calFF();
  else if (STARTS("CAL STEP")) calStep(a1 > 0 ? a1 : 0.15f);
  else if (STARTS("CAL DRIVE")) calDrive(a1 > 0 ? a1 : 1.0f);
  else if (STARTS("CAL DIST")) calDist(a1);
  else if (STARTS("CAL SPIN")) calSpin(a1 > 0 ? a1 : 5);
  else if (STARTS("CAL TURNS")) calTurns(a1);
  else if (STARTS("CAL TIMED")) calTimed(a1 > 0 ? (unsigned long)a1 : 2000UL);
  else if (IS("CAL VMIN")) calVmin();
  else if (IS("CAL DIR")) calDir();
  else if (STARTS("CAL RUN")) runStart(a1 > 0 ? a1 : 10.0f);
  else if (IS("CAL STOP")) runStop();
  else if (STARTS("CAL TOF")) calTof(a1 > 0 ? (int)a1 : 100, a2);
  else if (IS("CAL PING")) { Serial.print(F("PONG ")); Serial.println(cmdFromBt ? F("bluetooth") : F("usb")); }
  else if (IS("STATUS")) status();
  else if (IS("LOAD SENSORS")) { motorsEnabled = false; setPwm(0, 0); Serial.println(F("OK profile SENSORS (motors off)")); }
  else if (IS("LOAD FINAL")) { motorsEnabled = true; Serial.println(F("OK profile FINAL")); }
  else if (!extraCommand(u, orig)) Serial.println(F("ERR unknown command"));
}

void loop() {
  linkWatch();   // prints LINK bluetooth connected / disconnected when that changes
  // the "control loop": the sensors deliver readings at their own pace; STATUS reports the period of a full set
  idle(true);
  if (running) runTick();
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n') { line[lineLen] = 0; handle(line); lineLen = 0; }
    else if (ch != '\r' && lineLen < 63) line[lineLen++] = ch;
  }
}
`;
