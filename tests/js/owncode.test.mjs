// "I have my own code": the student prompt (protocol from firmware-gen.js, every number with its unit), the unit
// conversions, and the optional speed smoothing in the simulator (0 = the old results, bit for bit).
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { defaultRobot, robotToConfig } from '../../web/js/store.js';
import { exampleRobot } from '../../web/js/robotfile.js';
import { generateSketch, unoHardware, PROTOCOL, PROTOCOL_RULES, BAUD, protocolHeaderLines } from '../../web/js/firmware-gen.js';
import * as O from '../../web/js/own-code.js';
import { loadEngine, configText } from '../../web/js/engine.js';

const robots = () => {
  const a = exampleRobot();
  const b = exampleRobot(); b.hardware = unoHardware(); b.sensors.forEach((s) => { s.type = 'VL53L0X'; });
  const c = exampleRobot(); c.hardware.encoder.type = 'single'; c.hardware.board = 'mega';
  return [a, b, c];
};
// literal pieces of a reply line: the text between <placeholders>, without the trailing "   (explanation)"
function pieces(reply) {
  return reply.replace(/ {3}\(.*\)$/, '').split(/<[^>]*>/).map((x) => x.trim()).filter((x) => x.length >= 3);
}

test('every reply line in PROTOCOL is printed by the generated sketch', () => {
  for (const r of robots()) {
    const code = generateSketch(r, r.hardware).code;
    assert.ok(code.includes(`Serial.begin(${BAUD})`));
    for (const c of PROTOCOL) for (const rep of c.replies) for (const p of pieces(rep)) assert.ok(code.includes(p), `${c.cmd}: "${p}" not in the sketch`);
    for (const l of protocolHeaderLines()) assert.ok(code.includes(l), l);
  }
});

test('every command the sketch handles is in PROTOCOL, and every PROTOCOL command is handled', () => {
  const code = generateSketch(exampleRobot(), exampleRobot().hardware).code;
  const handled = new Set([...code.matchAll(/(?:IS|STARTS)\("([A-Z ]+)"\)/g)].map((m) => m[1]));
  const wire = (cmd) => (/^(CAL |STATUS|LOAD)/.test(cmd) ? cmd : 'CAL ' + cmd);   // RUN -> CAL RUN (the parser adds "CAL ")
  const listed = new Set(PROTOCOL.map((c) => wire(c.cmd)));
  for (const h of handled) assert.ok(listed.has(h), `sketch handles "${h}" but PROTOCOL does not list it`);
  for (const l of listed) assert.ok(handled.has(l), `PROTOCOL lists "${l}" but the sketch does not handle it`);
});

test('the student prompt holds the full protocol and every number with its unit', () => {
  for (const r of robots()) {
    r.motor.speed_tau_ms = 0;
    const txt = O.studentPrompt(r, r.hardware);
    assert.ok(txt.includes(`${BAUD} baud`));
    for (const rule of PROTOCOL_RULES) assert.ok(txt.includes(rule), rule);
    for (const c of PROTOCOL) {
      assert.ok(txt.includes(`- ${c.cmd}${c.args ? ' ' + c.args : ''}: ${c.help}`), c.cmd);
      for (const rep of c.replies) assert.ok(txt.includes('reply: ' + rep), rep);
    }
    const rows = O.numberTable(r);
    for (const k of ['pid.kp', 'pid.ki', 'pid.kd', 'pid.error_clip_mm', 'pid.integral_limit', 'pid.d_filter_alpha', 'pid.speed_frac',
      'motor.max_speed_mps', 'wheels.track_effective_mm', 'wheels.ticks_per_rev', 'motor.wheel_kp', 'motor.wheel_ki', 'motor.wheel_kd',
      'motor.ff_static', 'motor.ff_gain', 'motor.wheel_pid_period_ms', 'motor.speed_tau_ms']) assert.ok(rows.some((x) => x.path === k), k);
    for (const row of rows) {
      assert.ok(row.unit && row.unit.length > 1, row.path + ' has no unit');
      assert.ok(txt.includes(`- ${row.name} (${row.path}): ${+(+row.value).toPrecision(6)} ${row.unit}`), row.path);
    }
    assert.ok(txt.includes('No smoothing of the measured speed'));
  }
  const r = exampleRobot(); r.motor.speed_tau_ms = 30;
  assert.ok(O.studentPrompt(r, r.hardware).includes('time constant 30 ms'));
});

test('every serial command has a button (cmd-buttons.js) with an i18n label and help', async () => {
  const { readFileSync } = await import('node:fs');
  const src = readFileSync(new URL('../../web/js/cmd-buttons.js', import.meta.url), 'utf8');
  const en = JSON.parse(readFileSync(new URL('../../web/i18n/en.json', import.meta.url), 'utf8'));
  const btns = [...src.matchAll(/cmd: '([A-Z ]+)', id: '(\w+)'/g)];
  const cmds = new Set(btns.map((m) => m[1]));
  for (const c of PROTOCOL) assert.ok(cmds.has(c.cmd), `no button for ${c.cmd}`);
  for (const [, , k] of btns) { assert.ok(en['cmd.' + k], k); assert.ok(en['cmd.' + k + '_h'], k + '_h'); }
});

test('the prompt asks for a robot file and holds the current one as the template', async () => {
  const { robotToFile, fileToRobot, FILE_HEADER } = await import('../../web/js/robotfile.js');
  const { defaultRobot } = await import('../../web/js/store.js');
  for (const r of [exampleRobot(), defaultRobot()]) {
    const txt = O.studentPrompt(r, r.hardware);
    assert.ok(txt.includes('robot.robot.txt') && txt.includes(FILE_HEADER));
    const block = txt.slice(txt.indexOf('```\n' + FILE_HEADER) + 4, txt.lastIndexOf('```'));
    assert.equal(block.trimEnd(), robotToFile(r).trimEnd());          // the template = the student's robot file
    assert.deepEqual(fileToRobot(block).unknown, []);                   // and it loads as it is
  }
});

test('unit conversions round trip', () => {
  const close = (a, b, msg) => assert.ok(Math.abs(a - b) <= 1e-9 * Math.max(1, Math.abs(a), Math.abs(b)), `${msg}: ${a} vs ${b}`);
  const mpt = O.metersPerTick(45, 8291.5);
  for (const tp of [0, 1, 10, 91.5, -33]) {
    const v = O.tpToMps(tp, mpt, 10);
    close(O.mpsToTp(v, mpt, 10), tp, 'tp');
    close(O.rpmToMps(O.mpsToRpm(v, 45), 45), v, 'rpm');
    close(O.fracToMps(O.mpsToFrac(v, 0.156), 0.156), v, 'frac');
  }
  // worked example of the guide: 1 wheel turn per second = 60 RPM = pi x 45 mm per second
  close(O.mpsToRpm(Math.PI * 0.045, 45), 60, 'rpm example');
  close(O.mpsToTp(Math.PI * 0.045, mpt, 10), 82.915, 'tp example');
  const g = { kp: 2, ki: 0.001, kd: 5, ff_static: 4, ff_gain: 2.785 };
  const g50 = O.convertWheelGains(g, 10, 50), back = O.convertWheelGains(g50, 50, 10);
  for (const k of Object.keys(g)) close(back[k], g[k], 'period ' + k);
  close(g50.kp, 0.4, 'kp 50ms'); close(g50.kd, 0.2, 'kd 50ms'); close(g50.ff_gain, 0.557, 'ff 50ms');
  for (const unitPerTp of [mpt / 0.01, mpt / 0.01 / 0.156]) for (const out of [1, 1 / 255]) {
    const u = O.wheelGainsToUnit(g, unitPerTp, out), b = O.wheelGainsFromUnit(u, unitPerTp, out);
    for (const k of Object.keys(g)) close(b[k], g[k], 'unit ' + k);
  }
  // the same push: our Kp on a tick error = converted Kp on the same error in fraction of top speed
  const k = mpt / 0.01 / 0.156, frac = O.wheelGainsToUnit(g, k, 1 / 255);
  close(frac.kp * (10 * k) * 255, g.kp * 10, 'same PWM');
  close(O.kiSecondsToKiSum(O.kiSumToKiSeconds(0.001, 10), 10), 0.001, 'ki');
  close(O.kdSecondsToKdDiff(O.kdDiffToKdSeconds(5, 10), 10), 5, 'kd');
});

test('speed smoothing: 0 = the old simulator results exactly, > 0 changes them', async () => {
  const E = await loadEngine();
  const r = exampleRobot();
  const run = (extra) => {
    const sim = E.create();
    sim.config(configText(robotToConfig(r)) + extra);
    sim.walls([-1, 0.1, 3, 0.1, -1, -0.1, 3, -0.1]);
    sim.spawn(0, 0.01, 0.1); sim.reset(0, 0.01, 0.1, 7, 0);
    const out = [];
    for (let k = 0; k < 120; k++) { sim.step(1); out.push(...sim.stateRaw().slice(0, 12)); }
    sim.destroy();
    return out;
  };
  r.motor.speed_tau_ms = 0;
  const base = run(''), zero = run('wheel_speed_tau_ms 0\n');
  assert.ok(configText(robotToConfig(r)).includes('wheel_speed_tau_ms 0'));
  assert.deepEqual(zero, base);
  assert.notDeepEqual(run('wheel_speed_tau_ms 40\n'), base);
  // the native golden scenarios (wheel_model 1) do not set the key: they still pass in wasm_equiv.test.mjs
  assert.ok(!readFileSync(new URL('../golden/scenarios.txt', import.meta.url), 'utf8').includes('wheel_speed_tau_ms'));
});

test('every command says what the robot does (motion), and the prompt tells the AI', () => {
  const r = exampleRobot(), txt = O.studentPrompt(r, r.hardware);
  for (const c of PROTOCOL) {
    assert.ok(['hand', 'air', 'floor', 'still'].includes(c.motion), c.cmd);
    assert.ok(txt.includes(`[robot: ${c.motion}]`), c.cmd);
  }
  assert.ok(txt.includes('NO cable') && txt.includes('Never tell the student to drive the robot with the USB cable plugged in'));
});
