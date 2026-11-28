// The ESP32 install firmware: every setting it reads is a robot-file key, and the settings sent for a robot are complete.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { generateEsp32Install, FW_KEYS } from '../../web/js/firmware-esp32.js';
import { exampleRobot, allFields } from '../../web/js/robotfile.js';
import { settingsLines } from '../../web/js/install.js';
import { PROTOCOL } from '../../web/js/firmware-gen.js';

test('the firmware reads only robot-file keys, and they are all in FW_KEYS', () => {
  const code = generateEsp32Install().code;
  const read = new Set([...code.matchAll(/cfg(?:Get|F|Pin)\("([a-z0-9_]+)"/g)].map((m) => m[1]));
  for (const f of ['type', 'name', 'angle_deg', 'max_mm', 'min_mm', 'period_ms', 'xshut', 'addr', 'trig', 'echo', 'analog']) assert.ok(code.includes(`"sensor%d_${f}"`), f);
  const fileKeys = new Set(allFields(exampleRobot()).map((f) => f.key));
  const onlySome = ['pin_motor_left_en', 'pin_motor_right_en', 'pin_motor_stby', 'bluetooth_name', 'pid_offset_mm'];   // in a robot file only for some drivers / modes
  for (const k of read) { assert.ok(FW_KEYS.includes(k), `${k} not in FW_KEYS`); assert.ok(fileKeys.has(k) || onlySome.includes(k), `${k} is not a robot-file key`); }
  for (const k of FW_KEYS) if (!/^sensor\d_(trig|echo|analog|xshut|addr)$|^sensor[2-5]_/.test(k)) assert.ok(fileKeys.has(k) || onlySome.includes(k), `${k} is not a robot-file key`);
  // no generated-sketch leftovers; every measuring command of the protocol is there
  assert.ok(!/BT_BEGIN|USB_SERIAL_/.test(code));
  for (const c of PROTOCOL) assert.ok(code.includes(`"${/^(CAL |STATUS|LOAD)/.test(c.cmd) ? c.cmd : 'CAL ' + c.cmd}"`), c.cmd);
  assert.ok(code.includes('"CAL SET "') && code.includes('"CAL SAVE"') && code.includes('"CAL GET"'));
});

test('our robot sends every setting the firmware needs, none N/A', () => {
  const lines = settingsLines(exampleRobot());
  const keys = lines.map((l) => l.split(' ')[1]);
  for (const k of ['motor_driver', 'pin_motor_left_a', 'pin_motor_left_b', 'pin_motor_right_a', 'pin_motor_right_b', 'encoder_type', 'pin_enc_left_a',
    'enc_ticks_per_rev', 'wheel_diameter_mm', 'sensor_count', 'sensor1_type', 'sensor3_angle_deg', 'sensor2_xshut', 'pid_kp', 'max_speed_mps', 'bluetooth_name']) assert.ok(keys.includes(k), k);
  assert.ok(lines.every((l) => /^SET \S+ \S+$/.test(l) && !l.includes('N/A') && l.length <= 63), 'one short SET line each');
  assert.ok(lines.includes('SET pin_motor_left_a 25') && lines.includes('SET sensor1_addr 0x32'));
});
