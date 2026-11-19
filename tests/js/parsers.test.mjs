import { test } from 'node:test';
import assert from 'node:assert/strict';
import * as P from '../../web/js/measure-steps.js';

test('firmware calibration lines are parsed', () => {
  assert.deepEqual(P.parseTicks('CAL TICKS result: left 8290 right 8293 per revolution'), { left: 8290, right: 8293, ticks: 8291.5 });
  assert.equal(P.parseMaxSpeed('CAL MAXSPEED: left 0.156 m/s, right 0.157 m/s at PWM 255 (mean of the last 0.5 s)').max, 0.156);
  assert.deepEqual(P.parseSuggestion('CAL suggestion: SET ff_gain 2.785'), { key: 'ff_gain', value: 2.785 });
  assert.equal(P.parseFFSlope('CAL FF: slope 0.5570 PWM per tick/50ms (= 2.7850 per tick/10 ms)').perPeriod, 2.785);
  const s = P.parseStep('CAL STEP 0.10 m/s (= 3.7/3.7 ticks/period): rise L 0.120s R 0.130s, overshoot L 2% R 3%, tau63 L 0.080s R 0.090s, steady L 3.6 R 3.7');
  assert.ok(Math.abs(s.tau - 0.085) < 1e-9);
  assert.deepEqual(P.parseDist('CAL DIST: ticks per metre L 59000.0 R 59010.0; suggestions wscale_l 0.9720 wscale_r 0.9721 (CAL APPLY, SAVE)'), { scale_l: 0.972, scale_r: 0.9721 });
  assert.equal(P.parseSpinDone('CAL SPIN done: encoders say 10.000 turns (ticks L -1 R 1) with track_w 0.1200 m; asymmetry 0.4 %, at the PWM clamp 0 % of the time -> OK').trusted, true);
  assert.equal(P.parseTurns('CAL TURNS: effective track width 0.1249 m (was 0.1200). CAL APPLY, SAVE.').track_m, 0.1249);
  const t = P.parseTof('CAL TOF right n 200 mean 201.3 mm std 1.70 mm min 197 max 206 dropouts 0 (0.0%)');
  assert.equal(t.name, 'right'); assert.equal(t.std, 1.7);
  assert.equal(P.parseTofBias('CAL TOF front bias vs 200 mm: +7.0 mm').bias, 7);
  assert.equal(P.parseStatusLoop('control loop (ToF data-ready): period 25012 us (max 31000), compute 900 us, overruns 0, ticks 5, telemetry dropped 0').period_ms, 25.012);
  assert.deepEqual(P.parseStatusAges('ToF ages at the last tick: data-ready seen 3000/12000/8000 us ago (R/F/L), + 10 ms half budget').ages_ms, [13, 22, 18]);
});
test('sanity checks flag implausible values', () => {
  const robot = { wheels: { track_mm: 120 }, drive: '2wd' };
  assert.equal(P.sanity('spin', { track_m: 0.3 }, robot)[0], 'bad');
  assert.equal(P.sanity('spin', { track_m: 0.125 }, robot), null);
  assert.equal(P.sanity('ticks', { ticks: 8291, left: 8290, right: 8293 }), null);
});
