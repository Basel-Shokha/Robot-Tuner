// The tutorial's graph "distance from the centre vs distance driven" behaves as the text says.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { corridorCurve, settleDistance, GRAPH_PRESETS, wheelCommands, simulate } from '../../web/js/tutorial-sim.js';

const late = (c) => Math.max(...c.y.slice(-Math.floor(c.y.length / 4)).map(Math.abs));
const crossings = (c) => c.y.filter((v, i) => i && Math.sign(v) !== Math.sign(c.y[i - 1])).length;

test('starts 5 cm off centre and covers 1 m', () => {
  const c = corridorCurve(GRAPH_PRESETS.good);
  assert.ok(Math.abs(c.y[0] - 5) < 0.1);
  assert.ok(c.x.at(-1) > 95 && c.x.at(-1) <= 100);
});
test('good gains settle fast without much overshoot', () => {
  const c = corridorCurve(GRAPH_PRESETS.good);
  assert.ok(settleDistance(c) < 40);
  assert.ok(Math.min(...c.y) > -1);
});
test('Kp too large wobbles around the centre', () => {
  const c = corridorCurve(GRAPH_PRESETS.kp_high);
  assert.ok(crossings(c) >= 4);
  assert.ok(Math.min(...c.y) < -3);
});
test('P only keeps oscillating (and grows)', () => {
  const c = corridorCurve(GRAPH_PRESETS.p_only);
  assert.equal(settleDistance(c), Infinity);
  assert.ok(late(c) > 4);
});
test('Kd too large creeps slowly without crossing', () => {
  const c = corridorCurve(GRAPH_PRESETS.kd_high);
  assert.equal(crossings(c), 0);
  assert.equal(settleDistance(c), Infinity);
  assert.ok(c.y.at(-1) > 1);
});
test('demo 3: Kd = 0 clearly overshoots, a little Kd settles', () => {
  const p0 = simulate({ kp: 1, kd: 0, sensor_deg: 90, y0: 0.04 });
  assert.ok(Math.min(...p0.y) < -30);
  const pd = simulate({ kp: 1, kd: 0.5, sensor_deg: 90, y0: 0.04 });
  assert.ok(Math.min(...pd.y) > -10 && Math.abs(pd.y.at(-1)) < 1);
});
test('wheel commands: base -/+ u and clipping', () => {
  const w = wheelCommands(0.02, { vmin: 0.05, vmax: 0.3, frac: 0.75 });
  assert.ok(Math.abs(w.base - 0.2375) < 1e-12);
  assert.ok(Math.abs(w.left - 0.2175) < 1e-12 && Math.abs(w.right - 0.2575) < 1e-12);
  assert.equal(w.clipped, false);
  const c = wheelCommands(0.1, { vmin: 0.05, vmax: 0.3, frac: 0.75 });
  assert.equal(c.clipped, true);
  assert.equal(c.rightClip, 0.3);
});
