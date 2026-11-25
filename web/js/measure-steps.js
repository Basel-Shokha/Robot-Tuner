// measure-steps.js - parsers for the firmware's calibration answers (Simulation-To-Real-Life
// firmware/robot_fw_run25_nnmap, and firmware/robot_tuner_min which prints the same lines), plus sanity checks.
// Pure functions: tests/js/parsers.test.mjs feeds them real output lines.
const num = '([-+]?\\d+(?:\\.\\d+)?(?:[eE][-+]?\\d+)?)';
const R = (s) => new RegExp(s.replace(/#/g, num));

export function parseTicks(line) {  // "CAL TICKS result: left 8290 right 8293 per revolution"
  const m = R('CAL TICKS result: left # right #').exec(line); return m ? { left: +m[1], right: +m[2], ticks: (+m[1] + +m[2]) / 2 } : null;
}
export function parseMaxSpeed(line) {  // "CAL MAXSPEED: left 0.156 m/s, right 0.157 m/s at PWM 255 ..."
  const m = R('CAL MAXSPEED: left # m/s, right # m/s').exec(line); return m ? { left: +m[1], right: +m[2], max: Math.min(+m[1], +m[2]) } : null;
}
export function parseSuggestion(line) {  // "CAL suggestion: SET ff_gain 2.785"
  const m = /CAL suggestion: SET (\w+) ([-+\d.eE]+)/.exec(line); return m ? { key: m[1], value: +m[2] } : null;
}
export function parseFFSlope(line) {  // "CAL FF: slope 0.5570 PWM per tick/50ms (= 2.7850 per tick/10 ms)"
  const m = R('CAL FF(?: REV)?: slope # PWM per tick/50ms \\(= # per tick/(\\d+) ms\\)').exec(line); return m ? { per50: +m[1], perPeriod: +m[2], period: +m[3] } : null;
}
export function parseStep(line) {  // "CAL STEP 0.10 m/s (= ...): rise L 0.120s R 0.130s, overshoot L 2% R 3%, tau63 L 0.080s R 0.090s, steady L 9.8 R 9.7"
  const m = R('CAL STEP # m/s.*rise L #s R #s, overshoot L #% R #%, tau63 L #s R #s').exec(line);
  return m ? { target: +m[1], riseL: +m[2], riseR: +m[3], overL: +m[4], overR: +m[5], tauL: +m[6], tauR: +m[7], tau: (+m[6] + +m[7]) / 2 } : null;
}
// "CAL VMIN: left fwd 0.0210 m/s at PWM 36, right fwd 0.0190 m/s at PWM 39, left rev none, right rev 0.0200 m/s at PWM 42"
export function parseVmin(line) {
  if (!/^CAL VMIN:/.test(line.trim())) return null;
  const out = {};
  for (const m of line.matchAll(/(left|right) (fwd|rev) (none|([-+\d.eE]+) m\/s at PWM (\d+))/g)) out[`${m[1]}_${m[2]}`] = m[3] === 'none' ? null : { v: +m[4], pwm: +m[5] };
  const vals = Object.values(out);
  if (vals.length !== 4) return null;
  const ok = vals.filter(Boolean);
  return { wheels: out, vmin: ok.length === 4 ? Math.max(...ok.map((x) => x.v)) : null, missing: 4 - ok.length,
    lo: ok.length ? Math.min(...ok.map((x) => x.v)) : null };
}
export function parseTimed(line) {  // "CAL TIMED done: full power for 2000 ms at PWM 255. Measure the distance D (m) ..."
  const m = /CAL TIMED done: full power for (\d+) ms/.exec(line); return m ? { ms: +m[1] } : null;
}
/** top speed from a timed full-power run: the first ~tau seconds are spent accelerating (first-order lag) */
export function timedSpeed(distM, ms, tauS) { return distM / Math.max(0.2, ms / 1000 - Math.max(0, tauS || 0)); }
export function parseDriveDone(line) { return /CAL DRIVE done/.test(line) ? { done: true, sat: /WARNING/.test(line) } : null; }
export function parseDist(line) {  // "CAL DIST: ticks per metre L 59000.0 R 59010.0; suggestions wscale_l 0.9720 wscale_r 0.9721 ..."
  const m = R('suggestions wscale_l # wscale_r #').exec(line); return m ? { scale_l: +m[1], scale_r: +m[2] } : null;
}
export function parseSpinDone(line) {  // "CAL SPIN done: encoders say 5.000 turns ... asymmetry 1.2 %, at the PWM clamp 0 % of the time -> OK"
  const m = R('CAL SPIN done: encoders say # turns.*asymmetry # %').exec(line); return m ? { turns: +m[1], asym: +m[2], trusted: /-> OK/.test(line) } : null;
}
export function parseTurns(line) {  // "CAL TURNS: effective track width 0.1249 m (was 0.1200). CAL APPLY, SAVE."
  const m = R('CAL TURNS: effective track width # m').exec(line); return m ? { track_m: +m[1] } : null;
}
export function parseTof(line) {  // "CAL TOF right n 200 mean 201.3 mm std 1.70 mm min 197 max 206 dropouts 0 (0.0%)"
  const m = R('CAL TOF (\\w+)\\s+n (\\d+) mean # mm std # mm min (\\d+) max (\\d+) dropouts (\\d+) \\(#%\\)').exec(line);
  return m ? { name: m[1], n: +m[2], mean: +m[3], std: +m[4], min: +m[5], max: +m[6], drop: +m[7], dropPct: +m[8] } : null;
}
export function parseTofBias(line) {  // "CAL TOF front bias vs 200 mm: +7.0 mm"
  const m = R('CAL TOF (\\w+)\\s+bias vs # mm: # mm').exec(line); return m ? { name: m[1], true_mm: +m[2], bias: +m[3] } : null;
}
export function parseStatusLoop(line) {  // "control loop (ToF data-ready): period 25012 us (max 31000), compute 900 us, ..."
  const m = R('control loop.*period # us \\(max #\\)').exec(line); return m ? { period_ms: +m[1] / 1000, max_ms: +m[2] / 1000 } : null;
}
export function parseStatusAges(line) {  // "ToF ages at the last tick: data-ready seen 3000/12000/8000 us ago (R/F/L), + 10 ms half budget"
  const m = R('data-ready seen #/#/# us ago.*\\+ # ms half budget').exec(line);
  return m ? { ages_ms: [+m[1] / 1000 + +m[4], +m[2] / 1000 + +m[4], +m[3] / 1000 + +m[4]] } : null;
}

/** aggregate STATUS samples into loop period and reading age (mean) and spread (max - min) */
export function summarizeTiming(periods, ages) {
  const mean = (a) => a.reduce((s, x) => s + x, 0) / a.length;
  const flat = ages.flat();
  return { loop_ms: +mean(periods).toFixed(2), age_ms: +mean(flat).toFixed(2), age_jitter_ms: +(Math.max(...flat) - Math.min(...flat)).toFixed(2) };
}

// ---- sanity checks: [level, i18n key, vars] or null ---------------------------------------------------
export function sanity(step, v, robot) {
  switch (step) {
    case 'ticks':
      if (v.ticks < 20 || v.ticks > 100000) return ['bad', 'chk.ticks_range', { v: v.ticks }];
      if (Math.abs(v.left - v.right) / v.ticks > 0.02) return ['warn', 'chk.ticks_lr', { l: v.left, r: v.right }];
      return null;
    case 'maxspeed':
      if (v.max < 0.03 || v.max > 3) return ['bad', 'chk.speed_range', { v: v.max }];
      if (Math.abs(v.left - v.right) / v.max > 0.1) return ['warn', 'chk.speed_lr', { l: v.left, r: v.right }];
      return null;
    case 'vmin': {
      // the slowest speed every wheel drives steadily: must be well below the top speed, and the four values alike
      const vmax = robot.motor.max_speed_mps;
      if (v.vmin === null || v.missing) return ['bad', 'chk.vmin_missing', { n: v.missing }];
      if (!(v.vmin > 0) || v.vmin >= vmax) return ['bad', 'chk.vmin_range', { v: v.vmin, max: vmax }];
      if (v.vmin > 0.5 * vmax) return ['warn', 'chk.vmin_high', { v: v.vmin, max: vmax }];
      if (v.lo > 0 && v.vmin / v.lo > 2) return ['warn', 'chk.vmin_spread', { lo: v.lo, hi: v.vmin }];
      return null;
    }
    case 'step':
      if (v.tau <= 0 || v.tau > 1) return ['bad', 'chk.tau_range', { v: v.tau }];
      return null;
    case 'spin': {
      const mm = v.track_m * 1000, geo = robot.wheels.track_mm;
      if (mm < 0.8 * geo || mm > (robot.drive === '4wd' ? 2.2 : 1.35) * geo) return ['bad', 'chk.track', { v: mm.toFixed(0), g: geo }];
      return null;
    }
    case 'tof':
      if (v.std > 15) return ['warn', 'chk.tof_std', { n: v.name, v: v.std }];
      if (v.dropPct > 10) return ['warn', 'chk.tof_drop', { n: v.name, v: v.dropPct }];
      return null;
    case 'timing':
      if (v.loop_ms > 150) return ['warn', 'chk.loop_slow', { v: v.loop_ms }];
      if (v.age_ms > 3 * v.loop_ms) return ['warn', 'chk.age_old', { v: v.age_ms }];
      return null;
    default: return null;
  }
}
