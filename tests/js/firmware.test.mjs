// The sketch generator (web/js/firmware-gen.js): the real robot's sketch, pin checks, the serial protocol, and
// every combination tools/gen_firmware_combos.mjs hands to arduino-cli (the compile itself runs in CI).
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { defaultRobot, hardwareTxtToRobot, migrate, makeSensor } from '../../web/js/store.js';
import { exampleRobot, robotToFile, fileToRobot } from '../../web/js/robotfile.js';
import { generateSketch, validateHardware, hasErrors, suggestPins, unoHardware, defaultHardware, normalizeHardware, BOARDS, requiredLibraries } from '../../web/js/firmware-gen.js';
import { allCombos, buildCombo } from '../../tools/gen_firmware_combos.mjs';

const root = new URL('../../', import.meta.url);
const read = (p) => readFileSync(new URL(p, root), 'utf8');
const keys = (txt) => Object.fromEntries(txt.split('\n').map((l) => l.replace(/#.*/, '').trim()).filter(Boolean).map((l) => l.split(/\s+/)).map(([k, v]) => [k, v]));
const fx = (x) => +x.toFixed(4);
const lvl = (list, level) => list.filter((m) => m.level === level).map((m) => m.key);

test('ESP32 + quadrature + DRV8833 + 3x VL53L1X sketch matches the real robot (vendor robot_hardware.txt)', () => {
  const txt = read('vendor/simulation-to-real-life/robot_hardware.txt');
  const k = keys(txt), n = (key) => Number(k[key]);
  // start from a robot wired completely differently, so every pin below must come from the file
  const base = exampleRobot(); base.hardware = unoHardware();
  const r = hardwareTxtToRobot(txt, base);
  // the configuration the student picks in the builder: ESP32, quadrature encoders, DRV8833 (two-input driver)
  r.hardware.board = 'esp32'; r.hardware.encoder.type = 'quadrature'; r.hardware.driver.type = 'in1_in2';
  // not in the vendor file: brake (slow decay) and the N20 motor's 7 PPR x 296.125 gear (= the measured 8291.5 ticks / 28)
  r.hardware.driver.stop = 'brake'; r.hardware.encoder.ppr = 7; r.hardware.encoder.gear = 296.125;
  assert.deepEqual(r.sensors.map((s) => s.type), ['VL53L1X_short', 'VL53L1X_short', 'VL53L1X_short']);
  const g = generateSketch(r, r.hardware);
  const c = g.code;
  assert.equal(g.fqbn, 'esp32:esp32:esp32');
  // motor driver: DRV8833 IN1 = the pin upstream calls "pwm", IN2 = "dir"
  assert.ok(c.includes(`const int PIN_IN1_L = ${n('pin_motor_left_pwm')}, PIN_IN2_L = ${n('pin_motor_left_dir')}, PIN_IN1_R = ${n('pin_motor_right_pwm')}, PIN_IN2_R = ${n('pin_motor_right_dir')};`), 'motor pins');
  assert.ok(c.includes(`const int MOTOR_SIGN_L = ${n('motor_left_sign')}, MOTOR_SIGN_R = ${n('motor_right_sign')};`), 'motor signs');
  // encoders on the ESP32 hardware counter
  assert.ok(c.includes(`const int PIN_ENC_L_A = ${n('pin_enc_left_a')}, PIN_ENC_L_B = ${n('pin_enc_left_b')}, PIN_ENC_R_A = ${n('pin_enc_right_a')}, PIN_ENC_R_B = ${n('pin_enc_right_b')};`), 'encoder pins');
  assert.ok(c.includes(`const int ENC_SIGN_L = ${n('enc_left_sign')}, ENC_SIGN_R = ${n('enc_right_sign')};`), 'encoder signs');
  assert.ok(c.includes('#include <driver/pulse_cnt.h>') && c.includes('pcntL = pcntQuad(PIN_ENC_L_A, PIN_ENC_L_B)'), 'PCNT');
  assert.ok(c.includes(`float ticks_per_rev = ${n('enc_ticks_per_rev')}f;`), 'ticks per rev');
  assert.ok(c.includes(`const float WHEEL_DIAMETER_M = ${fx(n('wheel_diameter_mm') / 1000)}f;`), 'wheel diameter');
  assert.ok(c.includes(`float track_w = ${fx(n('track_w_effective_mm') / 1000)}f;`), 'track width');
  // I2C + three VL53L1X: XSHUT pins, addresses, short mode, budget and period
  assert.ok(c.includes(`const int PIN_SDA = ${n('pin_i2c_sda')}, PIN_SCL = ${n('pin_i2c_scl')};`), 'I2C pins');
  ['right', 'front', 'left'].forEach((name, i) => {
    const x = n(`pin_xshut_${name}`), a = '0x' + n(`tof_addr_${name}`).toString(16).toUpperCase();
    assert.ok(c.includes(`// sensor ${i} "${name}": VL53L1X_short, XSHUT GPIO${x}, address ${a}`), `sensor ${name}`);
    assert.ok(c.includes(`pinMode(${x}, INPUT); delay(10);`) && c.includes(`tof${i}.setAddress(${a});`), `wake + address ${name}`);
    assert.ok(c.includes(`tof${i}.setDistanceMode(VL53L1X::Short);`), `short mode ${name}`);
    assert.ok(c.includes(`tof${i}.setMeasurementTimingBudget(${n('tof_timing_budget_ms') * 1000});`) && c.includes(`tof${i}.startContinuous(${n('tof_period_ms')});`), `timing ${name}`);
  });
  assert.ok(c.includes(`const int XSHUT_PIN[] = {${n('pin_xshut_right')}, ${n('pin_xshut_front')}, ${n('pin_xshut_left')}};`));
  assert.deepEqual(requiredLibraries(r, r.hardware).map((l) => l.name), ['VL53L1X']);
  // the pins are legal; GPIO 12 and 5 are strapping pins and get a warning
  const v = validateHardware(r, r.hardware);
  assert.deepEqual(lvl(v, 'bad'), []);
  assert.ok(lvl(v, 'warn').includes('fw.v.strap12') && lvl(v, 'warn').includes('fw.v.strap'));
  // the site's example robot is this robot: same sketch
  const d = exampleRobot();
  assert.equal(generateSketch(d, d.hardware).code, c);
});

test('the committed example sketches are the generator output (node tools/gen_example_firmware.mjs)', () => {
  const d = exampleRobot(), code = generateSketch(d, d.hardware).code;
  assert.equal(read('firmware/robot_tuner_min/robot_tuner_min.ino'), code);
  assert.equal(read('web/firmware/robot_tuner_min.ino'), code);
});

test('the serial protocol: every line measure-steps.js parses is still printed', () => {
  for (const combo of allCombos().filter((c, i) => i % 7 === 0)) {
    const { robot, hw } = buildCombo(combo.board, combo.enc, combo.drv, combo.set);
    const c = generateSketch(robot, hw).code;
    for (const s of ['CAL TICKS result: left ', '" right "', 'CAL suggestion: SET ticks_rev ', 'CAL MAXSPEED: left ', ' m/s, right ', 'CAL suggestion: SET max_speed ',
      'CAL FF: slope ', ' PWM per tick/50ms (= ', 'CAL suggestion: SET ff_gain ', 'CAL suggestion: SET ff_static ', 'CAL: FF sweep done',
      'CAL STEP ', ' m/s (= sketch PI): rise L ', 's, overshoot L ', '%, tau63 L ', 'CAL DRIVE done: ticks L ', 'CAL DIST: ticks per metre L ', '; suggestions wscale_l ',
      'CAL SPIN done: encoders say ', ' %, at the PWM clamp 0 % of the time -> ', 'CAL TURNS: effective track width ', 'CAL TOF ', ' mm std ', ' dropouts ', 'CAL: TOF done',
      ' bias vs ', 'ToF ages at the last tick: data-ready seen ', ' us ago (R/F/L), + 0 ms half budget', 'control loop (ToF data-ready): period ', 'CAL TIMED done: full power for ',
      'Serial.begin(115200)', 'memcpy(u, "CAL ", 4)']) assert.ok(c.includes(s), `${combo.name}: ${s}`);
    assert.ok(!/undefined|NaN|\[object/.test(c), `${combo.name}: no undefined / NaN in the code`);
  }
});

test('every combination gets valid pins from suggestPins and generates', () => {
  const combos = allCombos();
  assert.ok(combos.length > 100);
  let ok = 0, refused = 0;
  for (const combo of combos) {
    const { robot, hw, failed } = buildCombo(combo.board, combo.enc, combo.drv, combo.set);
    if (failed.length) continue;  // too few pins on that board for that combination (reported by the tool)
    const bad = lvl(validateHardware(robot, hw), 'bad');
    if (combo.board === 'uno' && bad.length === 1 && bad[0] === 'fw.v.uno_two_tof') { refused++; continue; }   // too big for the Uno: refused on purpose
    assert.deepEqual(bad, [], combo.name);
    const g = generateSketch(robot, hw);
    assert.equal(g.fqbn, BOARDS[combo.board].fqbn);
    ok++;
  }
  assert.ok(ok + refused >= combos.length - 5, `${ok} + ${refused} refused of ${combos.length}`);
  assert.ok(refused > 0 && refused < 20, `${refused} Uno combinations refused (VL53L0X + VL53L1X)`);
});

test('pin checks catch the classic mistakes', () => {
  const r = exampleRobot();
  const check = (mod) => { const hw = normalizeHardware(JSON.parse(JSON.stringify(r.hardware)), r.sensors.length); mod(hw); return lvl(validateHardware(r, hw), 'bad').concat(lvl(validateHardware(r, hw), 'warn')); };
  assert.ok(check((h) => { h.driver.left.a = 34; }).includes('fw.v.input_only'), 'ESP32 34-39 cannot drive a motor');
  assert.ok(check((h) => { h.driver.left.a = 6; }).includes('fw.v.no_pin'), 'ESP32 flash pins');
  assert.ok(check((h) => { h.driver.left.a = 1; }).includes('fw.v.serial'), 'USB serial pin');
  assert.ok(check((h) => { h.driver.left.a = 32; }).includes('fw.v.twice'), 'pin used twice');
  assert.ok(check((h) => { h.encoder.left.a = 35; }).includes('fw.v.no_pullup'), 'input-only encoder pin: pull-up warning');
  assert.ok(check((h) => { h.sensors[1].xshut = -1; }).includes('fw.v.xshut_needed'), 'several ToF need XSHUT');
  assert.ok(check((h) => { h.sensors[1].addr = 0x30; }).includes('fw.v.addr_twice'), 'same I2C address');
  // Uno: x1 decoding needs A on D2 / D3; D4 is not an interrupt pin
  const u = exampleRobot(); u.hardware = unoHardware();
  assert.deepEqual(lvl(validateHardware(u, u.hardware), 'bad'), []);
  u.hardware.encoder.left.a = 4; u.hardware.encoder.left.b = 2;
  assert.ok(lvl(validateHardware(u, u.hardware), 'bad').includes('fw.v.no_int'));
  u.hardware.encoder.avr_mode = 'x4';  // with pin change interrupts any pin works
  assert.deepEqual(lvl(validateHardware(u, u.hardware), 'bad'), []);
  assert.deepEqual(requiredLibraries(u, u.hardware).map((l) => l.name), ['PinChangeInterrupt', 'VL53L1X']);
  // Uno: PWM pins only for IN1 / IN2, A4 / A5 are I2C
  const u2 = exampleRobot(); u2.hardware = unoHardware(); u2.hardware.driver.type = 'in1_in2';
  u2.hardware.driver.left = { a: 5, b: 8, en: -1 };
  assert.ok(lvl(validateHardware(u2, u2.hardware), 'bad').includes('fw.v.no_pwm'));
  u2.hardware.sensors[0].xshut = 18;
  assert.ok(lvl(validateHardware(u2, u2.hardware), 'bad').includes('fw.v.i2c_taken'));
  // two AS5600 on one bus need the multiplexer on AVR
  const u3 = exampleRobot(); u3.hardware = unoHardware(); u3.hardware.encoder.type = 'as5600'; u3.hardware.encoder.as5600_bus = 'two_buses';
  assert.ok(lvl(validateHardware(u3, u3.hardware), 'bad').includes('fw.v.as5600_two_buses_avr'));
});

test('encoder options: single channel, low resolution, AS5600, none', () => {
  const r = exampleRobot(); r.source = {};
  const gen = (enc) => { const hw = suggestPins(r, { ...r.hardware, encoder: { ...r.hardware.encoder, ...enc } }).hw; return { g: generateSketch(r, hw), hw }; };
  let { g } = gen({ type: 'single', ppr: 20, gear: 1 });
  assert.ok(g.lowRes && g.code.includes('#define LOW_RES 1') && g.code.includes('encL += cmdDirL') && g.code.includes('SINGLE-CHANNEL ENCODERS'));
  assert.equal(g.ticks, 20);
  ({ g } = gen({ type: 'quadrature', ppr: 7, gear: 296.125 }));
  assert.ok(!g.lowRes && g.code.includes('#define LOW_RES 0'));
  ({ g } = gen({ type: 'quadrature', ppr: 3, gear: 1 }));  // 12 ticks/rev: interrupts with edge times instead of PCNT
  assert.ok(g.lowRes && !g.code.includes('pulse_cnt') && g.code.includes('QDEC['));
  ({ g } = gen({ type: 'as5600', as5600_bus: 'mux', gear: 1 }));
  assert.equal(g.ticks, 4096);
  assert.ok(g.code.includes('muxSelect(MUX_CH_L)') && g.code.includes('if (d > 2048) d -= 4096'));
  ({ g } = gen({ type: 'as5600', as5600_bus: 'two_buses', gear: 1 }));
  assert.ok(g.code.includes('Wire1.begin(PIN_SDA2, PIN_SCL2, 400000)'));
  ({ g } = gen({ type: 'none' }));
  assert.ok(g.code.includes('#define HAS_ENCODERS 0') && g.code.includes('needs wheel encoders'));
  assert.ok(lvl(g.messages, 'warn').includes('fw.v.none'));
});

test('drivers: brake / coast, PWM + DIR, EN + IN1 + IN2 with STBY', () => {
  const r = exampleRobot();
  const gen = (drv) => generateSketch(r, suggestPins(r, { ...r.hardware, driver: { ...r.hardware.driver, ...drv } }).hw).code;
  assert.ok(gen({ type: 'in1_in2', stop: 'coast' }).includes('const bool STOP_BRAKE = false;'));
  const pd = gen({ type: 'pwm_dir' });
  assert.ok(pd.includes('PIN_PWM_L') && !pd.includes('STOP_BRAKE'));
  const en = gen({ type: 'en_in1_in2', stop: 'brake', stby: 99 });
  assert.ok(en.includes('PIN_EN_L') && en.includes('digitalWrite(PIN_STBY, HIGH)') && en.includes('const bool STOP_BRAKE = true;'));
});

test('sensors: HC-SR04 fire one at a time, Sharp curve, a single ToF needs no XSHUT', () => {
  const r = exampleRobot();
  r.sensors = [makeSensor('HC_SR04', { name: 'right', angle_deg: -90 }), makeSensor('GP2Y0A21', { name: 'front' }), makeSensor('VL53L0X', { name: 'left', angle_deg: 90 })];
  const { hw } = suggestPins(r, { ...r.hardware, sensors: [{}, {}, {}] });
  assert.equal(hw.sensors[2].xshut, -1);
  const v = validateHardware(r, hw);
  assert.deepEqual(lvl(v, 'bad'), []);
  assert.ok(lvl(v, 'warn').includes('fw.v.sonar') && lvl(v, 'warn').includes('fw.v.sonar_5v'));
  const c = generateSketch(r, hw).code;
  assert.ok(c.includes('sonarTurn = (sonarTurn + 1) % 1') && c.includes('pulseIn(SONAR_ECHO[k], HIGH, SONAR_TIMEOUT_US[k])'));
  assert.ok(c.includes('SHARP_A[] = {29.988f}') && c.includes('analogReadMilliVolts'));
  assert.ok(c.includes('#include <VL53L0X.h>') && !c.includes('XSHUT_PIN'));
  assert.ok(c.includes('const char* SENSOR_NAME[N_SENSORS] = {"right", "front", "left"};'));
});

test('hardware survives the saved robot and the robot file', () => {
  const r = exampleRobot(unoHardware());
  assert.deepEqual(migrate(JSON.parse(JSON.stringify(r))).hardware, r.hardware);
  // the robot file keeps every pin the sketch uses (Uno: I2C is fixed, so it is not in the file): same sketch
  for (const x of [r, exampleRobot()]) { const back = fileToRobot(robotToFile(x)).robot; assert.equal(generateSketch(back, back.hardware).code, generateSketch(x, x.hardware).code); }
  // a robot file from before the Firmware tab gets the example wiring
  const old = JSON.parse(JSON.stringify(exampleRobot())); delete old.hardware;
  assert.deepEqual(migrate(old).hardware, defaultHardware());
});
