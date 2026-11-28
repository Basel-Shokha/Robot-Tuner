import { test } from 'node:test';
import assert from 'node:assert/strict';
import { defaultRobot, robotToConfig, hardwareTxtToRobot, migrate, makeSensor } from '../../web/js/store.js';
import { exampleRobot, robotToFile, fileToRobot } from '../../web/js/robotfile.js';
import { loadEngine, configText } from '../../web/js/engine.js';
import { readFileSync } from 'node:fs';

test('every key the site sends is understood by the engine', async () => {
  const E = await loadEngine();
  const s = E.create();
  const r = exampleRobot();
  for (const variant of [r, { ...r, drive: '4wd', motor: { ...r.motor, model: 'lag' }, sensors: [...r.sensors, makeSensor('HC_SR04', { angle_deg: 90 }), makeSensor('GP2Y0A21', { angle_deg: -90 })] }]) {
    assert.equal(s.config(configText(robotToConfig(variant, { noise: 2, mismatch: 0.04, delayMs: 30 }))), 0);
  }
  s.destroy();
});

test('saved robot and robot file round trip', () => {
  const r = exampleRobot();
  assert.deepEqual(migrate(JSON.parse(JSON.stringify(r))), r);
  const back = fileToRobot(robotToFile(r)).robot;
  assert.equal(back.wheels.ticks_per_rev, r.wheels.ticks_per_rev);
  assert.equal(back.wheels.track_effective_mm, r.wheels.track_effective_mm);
  assert.ok(Math.abs(back.motor.wheel_kd - r.motor.wheel_kd) < 1e-6);
  assert.deepEqual(back.sensors.map((s) => [s.x_mm, s.y_mm, s.angle_deg]), r.sensors.map((s) => [+s.x_mm.toFixed(2), +s.y_mm.toFixed(2), s.angle_deg]));
});

test('our example robot = upstream robot_hardware.txt (mount description -> sensor poses)', () => {
  const txt = readFileSync(new URL('../../vendor/simulation-to-real-life/robot_hardware.txt', import.meta.url), 'utf8');
  const r = hardwareTxtToRobot(txt);
  assert.equal(r.wheels.ticks_per_rev, 8291.5);
  assert.equal(r.motor.max_speed_mps, 0.156);
  assert.ok(Math.abs(r.motor.wheel_kp - 2.0) < 1e-6, 'wheel_kp converted from 50 ms to 10 ms');
  assert.ok(Math.abs(r.sensors[0].x_mm - 100.76) < 0.05 && Math.abs(r.sensors[0].y_mm + 10.24) < 0.05, 'right ToF pose as gen_hardware.py derives it');
  assert.ok(Math.abs(r.sensors[2].x_mm - 98.64) < 0.05 && Math.abs(r.sensors[2].y_mm - 8.12) < 0.05, 'left ToF pose');
});
