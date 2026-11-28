// The robot file: every value written (N/A when not set), read back exactly, and the simulator gate.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { defaultRobot, migrate, isSet, robotToConfig } from '../../web/js/store.js';
import { robotToFile, fileToRobot, isRobotFile, allFields, missingForSim, exampleRobot, getPath } from '../../web/js/robotfile.js';
import { unoHardware } from '../../web/js/firmware-gen.js';

test('a new robot has nothing set: every value is N/A and the simulator asks for the required ones', () => {
  const r = defaultRobot(), txt = robotToFile(r);
  assert.ok(isRobotFile(txt));
  for (const f of allFields(r)) if (f.path !== 'sensors.length') assert.match(txt, new RegExp(`^${f.key} +N/A `, 'm'), f.key);
  const miss = missingForSim(r).map((f) => f.path);
  for (const p of ['body.front_mm', 'wheels.diameter_mm', 'wheels.track_mm', 'motor.max_speed_mps', 'timing.loop_ms', 'sensors.0.type', 'sensors.2.angle_deg']) assert.ok(miss.includes(p), p);
  assert.equal(robotToConfig(r).wheel_model, 0);   // no motor values set: the simple motor model
});

test('our robot: every value set, the file reads back to the same robot', () => {
  for (const r of [exampleRobot(), exampleRobot(unoHardware())]) {
    assert.deepEqual(missingForSim(r), []);
    const txt = robotToFile(r);
    assert.ok(!/^[a-z]\S* +N\/A /m.test(txt), 'no N/A in a complete robot');
    const { robot, unknown } = fileToRobot(txt);
    assert.deepEqual(unknown, []);
    for (const f of allFields(r)) {
      if (f.path === 'sensors.length') continue;
      const a = getPath(r, f.path), b = getPath(robot, f.path);
      if (typeof a === 'number') assert.ok(Math.abs(a - b) <= 1e-6 * Math.max(1, Math.abs(a)), `${f.key}: ${a} vs ${b}`); else assert.equal(b, a, f.key);
      assert.ok(isSet(robot, f.path), f.key + ' set after loading');
    }
    assert.deepEqual(robotToConfig(robot), robotToConfig(r));   // the simulator sees the same robot
    assert.equal(robotToFile(robot).split('\n').filter((l) => !l.startsWith('#')).map((l) => l.split('#')[0].trim()).join('\n'),
      txt.split('\n').filter((l) => !l.startsWith('#')).map((l) => l.split('#')[0].trim()).join('\n'));
  }
});

test('N/A lines stay unset, a set value is set, unknown lines are reported', () => {
  const txt = robotToFile(defaultRobot()).replace(/^wheel_diameter_mm +N\/A/m, 'wheel_diameter_mm 65') + 'mystery_key 3\n';
  const { robot, unknown } = fileToRobot(txt);
  assert.equal(robot.wheels.diameter_mm, 65); assert.ok(isSet(robot, 'wheels.diameter_mm'));
  assert.ok(!isSet(robot, 'wheels.track_mm'));
  assert.deepEqual(unknown, ['mystery_key']);
});

test('robots saved before robot files keep every value (and their simulation)', () => {
  const old = JSON.parse(JSON.stringify(exampleRobot())); delete old.v2; old.source = { 'wheels.ticks_per_rev': 'MEASURED' };
  const m = migrate(old);
  assert.deepEqual(missingForSim(m), []);
  assert.equal(robotToConfig(m).wheel_model, 1);
});

test('robots/basel_nicole.robot.txt is the generator output (node tools/gen_example_robot.mjs)', async () => {
  const { readFileSync } = await import('node:fs');
  assert.equal(readFileSync(new URL('../../robots/basel_nicole.robot.txt', import.meta.url), 'utf8'), robotToFile(exampleRobot()));
});
