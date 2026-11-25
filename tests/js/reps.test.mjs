// Repeated counting (N turns / N spins, then divide) on the Measure page.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import * as P from '../../web/js/measure-steps.js';

test('ticks per revolution from one count per turn', () => {
  const v = P.ticksFromReps([{ left: 8290, right: 8293 }, { left: 8292, right: 8291 }, { left: 8291, right: 8292 }, { left: 8289, right: 8290 }, { left: 8293, right: 8294 }]);
  assert.equal(v.n, 5);
  assert.equal(v.left, 8291);
  assert.equal(v.right, 8292);
  assert.equal(v.ticks, 8291.5);
  assert.ok(v.spread < 0.1);
  assert.equal(P.sanity('ticks', v), null);
});

test('total over N turns divided by N equals the mean of the per-turn counts', () => {
  const reps = [{ left: 360, right: 361 }, { left: 362, right: 359 }, { left: 358, right: 360 }, { left: 361, right: 362 }, { left: 359, right: 358 }, { left: 360, right: 360 }];
  const a = P.ticksFromReps(reps);
  const b = P.ticksFromTotals(reps.reduce((s, r) => s + r.left, 0), reps.reduce((s, r) => s + r.right, 0), reps.length);
  assert.ok(Math.abs(a.ticks - b.ticks) < 1e-9);
  assert.ok(Math.abs(a.left - b.left) < 1e-9 && Math.abs(a.right - b.right) < 1e-9);
  assert.equal(P.ticksFromTotals(100, 100, 0), null);
});

test('turns that disagree by more than 2 % are flagged', () => {
  const v = P.ticksFromReps([{ left: 1000, right: 1000 }, { left: 1000, right: 1000 }, { left: 1030, right: 1000 }, { left: 1000, right: 1000 }, { left: 1000, right: 1000 }]);
  assert.ok(v.spread > 2);
  const c = P.sanity('ticks', v);
  assert.equal(c[0], 'warn');
  assert.equal(c[1], 'chk.ticks_reps');
  // 1 % spread is fine
  assert.equal(P.sanity('ticks', P.ticksFromReps([{ left: 1000, right: 1000 }, { left: 1010, right: 1005 }])), null);
});

test('repetition counts are kept inside the allowed range', () => {
  assert.equal(P.clampReps(3, P.REPS.ticks), 5);
  assert.equal(P.clampReps(12, P.REPS.ticks), 10);
  assert.equal(P.clampReps('7', P.REPS.ticks), 7);
  assert.equal(P.clampReps(6.6, P.REPS.ticks), 7);
  assert.equal(P.clampReps('x', P.REPS.ticks), 5);
  assert.equal(P.REPS.spin.def, 10);
});

test('effective track width from N spins and the counted turns (fractions allowed)', () => {
  // the encoders say 10 turns with a 120 mm track, the student counted 9.8: the real track is wider
  const v = P.trackFromSpin(0.12, 10, 9.8);
  assert.ok(Math.abs(v.track_m - 0.12 * 10 / 9.8) < 1e-12);
  assert.ok(Math.abs(v.perTurnErrPct - 100 * 0.2 / 9.8) < 1e-9);
  assert.equal(P.trackFromSpin(0.12, 10, 10).track_m, 0.12);
  assert.equal(P.trackFromSpin(0.12, 10, 0), null);
  // same as the firmware's CAL TURNS: tw = track_w * spinEncRot / (real * 2 pi), spinEncRot = encTurns * 2 pi
  const fw = 0.12 * (10 * 2 * Math.PI) / (9.8 * 2 * Math.PI);
  assert.ok(Math.abs(v.track_m - fw) < 1e-12);
});

test('counted turns far from the commanded ones are rejected', () => {
  assert.ok(P.turnsPlausible(9.8, 10));
  assert.ok(!P.turnsPlausible(5, 10));
  assert.ok(!P.turnsPlausible(14, 10));
  assert.ok(!P.turnsPlausible(0, 10));
});
