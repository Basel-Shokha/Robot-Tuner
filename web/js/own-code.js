// own-code.js - "I have my own code": the units of every number the Tuner exports, unit conversions, and the prompt
// a student pastes into their AI so it adapts THEIR robot code to talk to the Tuner.
//
// Pure module (no DOM). The protocol in the prompt comes from firmware-gen.js PROTOCOL (the same table the sketch
// header is built from; tests/js/owncode.test.mjs checks every reply line is really printed by the sketch).
//
// Units are taken from the code, not guessed:
//   wheel PID  = vendor/.../firmware/robot_fw/wheel_pid.h (run by sim/rt_engine.hpp Drive::Step every
//                motor.wheel_pid_period_ms): speeds in encoder ticks PER PID PERIOD, output in PWM counts.
//   steering   = sim/rt_engine.hpp Pid::Control = export.js cFunction(): error in metres, output in m/s.
import { PROTOCOL, PROTOCOL_RULES, MOTION, usesBt, btName, BOARDS, DRIVER_TEXT, encoderText, edgesPerPulse, computedTicksPerRev, sketchTicks,
  normalizeHardware, pinRoles, pinLabel, sensorWireName, requiredLibraries } from './firmware-gen.js';
import { cruiseSpeed } from './store.js';
import { robotToFile, FILE_HEADER } from './robotfile.js';

const g4 = (x) => +(+x).toPrecision(4);
const g6 = (x) => +(+x).toPrecision(6);

// ---- conversions --------------------------------------------------------------------------------------------
/** metres the wheel rolls per encoder tick (= wheel_pid.h MetersPerTick with scale 1) */
export const metersPerTick = (diameterMm, ticksPerRev) => Math.PI * diameterMm / 1000 / ticksPerRev;
/** ticks per period <-> m/s (= wheel_pid.h TpToMps / MpsToTp) */
export const tpToMps = (tp, mpt, periodMs) => tp * mpt / (periodMs / 1000);
export const mpsToTp = (v, mpt, periodMs) => v * (periodMs / 1000) / mpt;
/** m/s <-> wheel revolutions per minute */
export const mpsToRpm = (v, diameterMm) => v / (Math.PI * diameterMm / 1000) * 60;
export const rpmToMps = (rpm, diameterMm) => rpm / 60 * Math.PI * diameterMm / 1000;
/** m/s <-> fraction of the top speed (0.5 = 50 %) */
export const mpsToFrac = (v, vmax) => v / vmax;
export const fracToMps = (fr, vmax) => fr * vmax;
/** Wheel PID gains from loop period A to period B, same real-time behaviour (= wheel_pid.h ConvertGains) */
export function convertWheelGains(g, periodAms, periodBms) {
  const r = periodBms / periodAms;
  return { kp: g.kp / r, ki: g.ki, kd: g.kd / (r * r), ff_static: g.ff_static, ff_gain: g.ff_gain / r };
}
/** Our wheel gains (error in ticks/period, output PWM counts) for code whose speed error is in another unit.
 *  unitPerTp = how many of THEIR speed units one tick/period is (m/s: mpt/period_s; fraction of max: mpt/period_s/vmax).
 *  outScale = their output for one PWM count (1 if they write PWM counts; 1/255 if their output is 0..1). */
export function wheelGainsToUnit(g, unitPerTp, outScale = 1) {
  return { kp: g.kp / unitPerTp * outScale, ki: g.ki / unitPerTp * outScale, kd: g.kd / unitPerTp * outScale,
    ff_static: g.ff_static * outScale, ff_gain: g.ff_gain / unitPerTp * outScale };
}
export function wheelGainsFromUnit(g, unitPerTp, outScale = 1) {
  return wheelGainsToUnit(g, 1 / unitPerTp, 1 / outScale);
}
/** a sum-of-errors integral (ours: once per period) <-> an integral of e dt in seconds; a difference <-> de/dt */
export const kiSumToKiSeconds = (ki, periodMs) => ki / (periodMs / 1000);
export const kiSecondsToKiSum = (ki, periodMs) => ki * (periodMs / 1000);
export const kdDiffToKdSeconds = (kd, periodMs) => kd * (periodMs / 1000);
export const kdSecondsToKdDiff = (kd, periodMs) => kd / (periodMs / 1000);

// ---- the numbers and their units ------------------------------------------------------------------------------
/** Every exported number: [path, value, unit, what 1.0 means]. Used by the guide page and the prompt. */
export function numberTable(robot) {
  const m = robot.motor, p = robot.pid, w = robot.wheels, P = m.wheel_pid_period_ms;
  const tp = `tick/${P}ms`;
  return [
    { group: 'steer', path: 'pid.kp', name: 'Steering Kp', value: p.kp, unit: '(m/s) per m', one: '1 cm more on the left than on the right -> each wheel 0.01 m/s faster/slower' },
    { group: 'steer', path: 'pid.ki', name: 'Steering Ki', value: p.ki, unit: '(m/s) per (m*s)', one: '1 cm error held for 1 s adds 0.01 m/s of turn' },
    { group: 'steer', path: 'pid.kd', name: 'Steering Kd', value: p.kd, unit: '(m/s) per (m/s) = s', one: 'error growing 1 cm per second -> 0.01 m/s of turn' },
    { group: 'steer', path: 'pid.error_clip_mm', name: 'Error clip', value: p.error_clip_mm, unit: 'mm', one: 'the error (left - right) is cut to +-1 mm' },
    { group: 'steer', path: 'pid.integral_limit', name: 'Integral limit', value: p.integral_limit, unit: 'm*s', one: 'the integral of the error stops at +-1 m*s' },
    { group: 'steer', path: 'pid.d_filter_alpha', name: 'D smoothing alpha', value: p.d_filter_alpha, unit: 'no unit, per 50 ms', one: '1.0 = no smoothing of the D term; 0.3 = 30 % of the new value per 50 ms' },
    { group: 'steer', path: 'pid.speed_frac', name: 'Speed fraction', value: p.speed_frac, unit: 'fraction 0..1', one: 'cruise at the max wheel speed (cruise = v_min + speed_frac x (v_max - v_min))' },
    { group: 'robot', path: 'motor.max_speed_mps', name: 'Max wheel speed (v_max)', value: m.max_speed_mps, unit: 'm/s', one: '1 metre per second at full PWM' },
    { group: 'robot', path: 'motor.v_min_mps', name: 'Slowest steady speed (v_min)', value: m.v_min_mps || 0, unit: 'm/s', one: '1 metre per second' },
    { group: 'robot', path: 'wheels.track_effective_mm', name: 'Track width (effective)', value: w.track_effective_mm, unit: 'mm', one: 'wheels 1 mm apart (from CAL SPIN, not the ruler)' },
    { group: 'robot', path: 'wheels.ticks_per_rev', name: 'Ticks per wheel revolution', value: w.ticks_per_rev, unit: 'ticks per wheel turn', one: 'one counted encoder edge per turn of the WHEEL' },
    { group: 'robot', path: 'wheels.diameter_mm', name: 'Wheel diameter', value: w.diameter_mm, unit: 'mm', one: '1 mm' },
    { group: 'robot', path: 'timing.loop_ms', name: 'Steering loop period', value: robot.timing.loop_ms, unit: 'ms', one: 'one PID step per fresh set of sensor readings' },
    { group: 'wheel', path: 'motor.wheel_pid_period_ms', name: 'Wheel loop period', value: P, unit: 'ms', one: 'the wheel PID runs every 1 ms' },
    { group: 'wheel', path: 'motor.wheel_kp', name: 'Wheel Kp', value: m.wheel_kp, unit: `PWM per ${tp}`, one: `wheel 1 tick per ${P} ms too slow -> +1 PWM` },
    { group: 'wheel', path: 'motor.wheel_ki', name: 'Wheel Ki', value: m.wheel_ki, unit: `PWM per ${tp}, per period (sum of errors)`, one: `1 tick per ${P} ms error for one period adds 1 PWM, forever` },
    { group: 'wheel', path: 'motor.wheel_kd', name: 'Wheel Kd', value: m.wheel_kd, unit: `PWM per (${tp} change per period)`, one: `error grows by 1 tick per ${P} ms from one period to the next -> 1 PWM` },
    { group: 'wheel', path: 'motor.ff_static', name: 'Feedforward static (start PWM)', value: m.ff_static, unit: 'PWM', one: '1 PWM count (of 255) added in the direction of the target' },
    { group: 'wheel', path: 'motor.ff_gain', name: 'Feedforward gain', value: m.ff_gain, unit: `PWM per ${tp}`, one: `target 1 tick per ${P} ms -> 1 PWM` },
    { group: 'wheel', path: 'motor.pwm_limit', name: 'PWM limit', value: m.pwm_limit, unit: 'PWM (0..255)', one: 'output clamped to +-1 PWM count' },
    { group: 'wheel', path: 'motor.speed_tau_ms', name: 'Speed smoothing', value: m.speed_tau_ms || 0, unit: 'ms (EMA time constant, 0 = off)', one: 'the measured speed follows a change with a 1 ms delay' },
  ];
}

// ---- the student prompt ---------------------------------------------------------------------------------------
function hardwareLines(robot, hwIn) {
  const hw = normalizeHardware(hwIn, robot.sensors.length), B = BOARDS[hw.board], ticks = sketchTicks(robot, hw), e = hw.encoder;
  const L = [];
  L.push(`- Board: ${B.label} (Arduino IDE board / arduino-cli FQBN ${B.fqbn}).`);
  L.push(`- Motor driver: ${DRIVER_TEXT[hw.driver.type]}. Stop mode: ${hw.driver.type === 'pwm_dir' ? 'decided by the driver board at PWM 0' : hw.driver.stop}.`);
  L.push(`- Motor sign: MOTOR_SIGN_L = ${hw.driver.sign_l < 0 ? -1 : 1}, MOTOR_SIGN_R = ${hw.driver.sign_r < 0 ? -1 : 1} (multiplied with the PWM so that positive PWM = wheel forward).`);
  L.push(`- Wheel encoders: ${encoderText(hw, ticks)}.`);
  if (e.type === 'quadrature' || e.type === 'single') L.push(`  Encoder: ${+e.ppr} pulses per motor-shaft turn, gear ratio ${+e.gear} (motor turns per wheel turn).`);
  if (e.type === 'as5600') L.push(`  AS5600: 4096 counts per magnet turn, gear ratio ${+e.gear}.`);
  if (e.type === 'quadrature' || e.type === 'as5600') L.push(`  Encoder sign: ENC_SIGN_L = ${e.sign_l < 0 ? -1 : 1}, ENC_SIGN_R = ${e.sign_r < 0 ? -1 : 1} (multiplied with the raw count so forward = positive).`);
  L.push(`- Wheels: diameter ${robot.wheels.diameter_mm} mm, track width (wheel centre to centre) ${robot.wheels.track_mm} mm, effective track width ${robot.wheels.track_effective_mm} mm. Drive: ${robot.drive}.`);
  L.push(`- Distance sensors (index: name, type, mounting angle; + = looking left, 0 = straight ahead):`);
  robot.sensors.forEach((s, i) => L.push(`  ${i}: "${sensorWireName(s, i)}", ${s.type}, angle ${s.angle_deg} deg, at x ${s.x_mm} mm / y ${s.y_mm} mm from the wheel axle centre, range ${s.min_mm}-${s.max_mm} mm.`));
  L.push('- Pins used by the reference sketch (keep the student\'s own pins if they differ, and tell the student):');
  for (const r of pinRoles(robot, hw)) { const v = r.get(); L.push(`  ${r.role}: ${v >= 0 ? pinLabel(hw.board, v) : 'not set'}`); }
  if (usesBt(hw)) L.push(`- Bluetooth serial: ON, device name "${btName(robot, hw)}" (classic ESP32, BluetoothSerial library from the esp32 board package). The robot must answer over USB and Bluetooth at the same time: copy the TeeIO / RT_IO bridge of the reference and send every reply through RT_IO.`);
  else if (hw.board === 'esp32') L.push('- Bluetooth serial: off (USB only).');
  const libs = requiredLibraries(robot, hw);
  L.push(`- Libraries: ${libs.length ? libs.map((l) => `"${l.name}" by ${l.by}`).join(', ') : 'only the built-in ones'}.`);
  return { lines: L, hw, ticks };
}

function decodingText(hw) {
  const e = hw.encoder, n = edgesPerPulse(hw);
  if (e.type === 'quadrature') return n === 1 ? 'x1: one tick per RISING edge of channel A; channel B read at that moment gives the direction'
    : 'x4: one tick for EVERY edge (rising and falling) of BOTH channels A and B, direction from the A/B state table';
  if (e.type === 'single') return 'x1 on one channel: one tick per rising edge; the sign comes from the last commanded motor direction';
  if (e.type === 'as5600') return '4096 ticks per magnet turn (12-bit angle; turns counted from the wrap through 0)';
  return 'no encoders';
}

/** The prompt text (English, written for an AI reader). opts.sketchName = the reference file name. */
export function studentPrompt(robot, hwIn, opts = {}) {
  const ref = opts.sketchName || 'robot_tuner_reference.ino';
  const { lines: hwLines, hw, ticks } = hardwareLines(robot, hwIn);
  const m = robot.motor, p = robot.pid, P = m.wheel_pid_period_ms;
  const mpt = metersPerTick(robot.wheels.diameter_mm, robot.wheels.ticks_per_rev);
  const vmax = m.max_speed_mps, vmin = Math.min(Math.max(0, m.v_min_mps || 0), vmax);
  const tau = Math.max(0, +m.speed_tau_ms || 0);
  const angles = robot.sensors.map((s) => s.angle_deg);
  const li = angles.indexOf(Math.max(...angles)), ri = angles.indexOf(Math.min(...angles));
  const robotFileTemplate = robotToFile(robot);
  const T = [];
  const w = (...x) => T.push(...x);
  w('# Task: connect a student\'s own robot code to Robot Tuner', '',
    'You are helping a student connect their own robot code to Robot Tuner (a website that measures the robot over USB serial and tunes its PID gains in a simulator).',
    '', 'Attached:',
    '1. The student\'s own robot code (all files).',
    `2. ${ref} - generated by Robot Tuner for the same hardware. It is the reference: it shows exactly what the Tuner expects.`,
    '', '## What to do',
    'Add to the student\'s code everything the reference uses to talk to the Tuner, without breaking the student\'s own logic:',
    '- the serial protocol (section A): baud rate, every command and every reply, with the exact text,',
    '- encoder counting and its sign convention (section B),',
    '- the wheel speed loop (section C),',
    '- the steering (wall-following) controller (section D).',
    '', 'Rules:',
    '- Keep the student\'s pins, names and structure. Change as little as possible.',
    '- Ask the student for anything that is missing or unclear. Do NOT guess pins, signs, units or loop periods.',
    '- If the student\'s code uses other units or another loop period, convert every gain with the formulas in section F BEFORE using it.',
    '- Use Serial.print / Serial.println, no String class in the command parser (it must also fit an Arduino Uno).',
    '- Do not block loop() for long: the encoders and the sensors must keep running while a command runs (see idle() / waitMs() in the reference).',
    '- Give back: (1) the COMPLETE code (every file), ready to upload in the Arduino IDE, (2) a short list of what you changed and why, (3) the robot file robot.robot.txt (section G).',
    '');

  w('## Hardware (from the student\'s Robot Tuner profile)', ...hwLines, '');

  w('## A. Serial protocol (copy exactly; the Tuner reads these lines)', 'General rules:', ...PROTOCOL_RULES.map((r) => '- ' + r), '',
    'Commands. "<x>" = a number filled in at run time; [x] = optional argument. Each reply is one line, printed exactly as written (same words, spaces, units and punctuation).');
  for (const c of PROTOCOL) {
    w(`- ${c.cmd}${c.args ? ' ' + c.args : ''}: ${c.help}${c.def ? ` (default ${c.def})` : ''}${c.enc ? ' [needs encoders]' : ''} [robot: ${c.motion}]`);
    for (const r of c.replies) w(`    reply: ${r}`);
  }
  w('Robot states in [robot: ...]:', ...Object.entries(MOTION).map(([k, v]) => `- ${k}: ${v}`),
    'Commands arrive over USB or, with the robot on the floor, over Bluetooth (no cable). The floor commands (CAL DRIVE, CAL SPIN, CAL TIMED, RUN) must stop on their own (distance, turns or time), STOP must work at any moment, and they must not wait for anything after they start. Never tell the student to drive the robot with the USB cable plugged in.');
  w('Follow the reference for HOW each command measures (timings, PWM steps, averaging); the Tuner\'s checks assume the same procedure.', '');

  w('## B. Encoder counting',
    `- Count signed ticks per wheel. FORWARD = POSITIVE for both wheels (apply the encoder sign ENC_SIGN_* so this holds).`,
    `- Decoding: ${decodingText(hw)}.`,
    `- Ticks per WHEEL revolution: ${g6(ticks)} (computed: ${hw.encoder.type === 'none' ? 'no encoders' : `${hw.encoder.type === 'as5600' ? 4096 : +hw.encoder.ppr} x ${hw.encoder.type === 'as5600' ? 1 : edgesPerPulse(hw)} edges x gear ${+hw.encoder.gear} = ${g6(computedTicksPerRev(hw))}`}). The student must confirm it with CAL TICKS (turn the wheel once by hand) and use the measured value.`,
    `- If the student's code decodes differently (x1 / x2 / x4), ticks per revolution changes by the same factor: x2 = 2 ticks per pulse, x4 = 4 ticks per pulse. Every gain in ticks must then be converted (section F).`,
    `- Metres per tick = pi x wheel diameter / ticks per revolution = ${g6(mpt * 1000)} mm.`, '');

  w('## C. Wheel speed loop (one per wheel)',
    `The Tuner simulates this loop (Robot Tuner firmware wheel_pid.h); its numbers below are only valid for it:`,
    `- Runs every ${P} ms. Speed unit: encoder ticks per ${P} ms ("tp"). Output: PWM counts -255..255, positive = forward.`,
    '- target_tp = target_mps x period_s / metres_per_tick;  meas_tp = ticks counted in the last period' + (tau > 0 ? ', then smoothed (see below)' : '') + '.',
    '- If |target_tp| < 0.01: PWM = 0 and reset (integral = 0, previous error = 0).',
    '- e = target_tp - meas_tp',
    '- ff = sign(target) x FF_STATIC + FF_GAIN x target_tp',
    '- trial_integral = integral + e   (a sum of errors, one per period; NOT multiplied by dt)',
    '- u = ff + KP x e + KI x trial_integral + KD x (e - previous_e)',
    `- clamp u to +-PWM_LIMIT (${m.pwm_limit}); only when u was NOT clamped: integral = trial_integral (anti-windup). previous_e = e. PWM = round(u).`,
    `- Gains: KP ${g6(m.wheel_kp)} PWM per tp, KI ${g6(m.wheel_ki)} PWM per tp per period, KD ${g6(m.wheel_kd)} PWM per (tp change per period), FF_STATIC ${g6(m.ff_static)} PWM, FF_GAIN ${g6(m.ff_gain)} PWM per tp, PWM_LIMIT ${m.pwm_limit} PWM.`,
    tau > 0 ? `- Speed smoothing: meas_s += a x (meas_tp - meas_s) with a = 1 - exp(-${P} / ${tau}) = ${g4(1 - Math.exp(-P / tau))} (time constant ${tau} ms). The gains were tuned WITH this smoothing; keep it.`
      : '- No smoothing of the measured speed (the gains were tuned without it). If the student\'s code smooths its speed reading, tell the student to set "Speed smoothing" in the Tuner\'s Robot builder to the same time constant and tune again.',
    `Note: the reference sketch has its own, simpler wheel PI in m/s (piOne(): u = sign x ff_static + ff_gain x target + wkp x e + wki x integral of e dt, with wkp 300 PWM per (m/s), wki 2000 PWM per m, every 10 ms). Its calibration commands (CAL STEP / DRIVE / SPIN, RUN) use it. Keep it for those commands; use the loop above for the student's own driving.`,
    '');

  w('## D. Steering controller (wall following, constant cruise speed)',
    `Runs once per fresh set of sensor readings (about every ${robot.timing.loop_ms} ms on this robot). dt = measured time since the last step, in seconds.`,
    `Inputs: left_m = distance of the sensor looking most to the left (index ${li}, "${sensorWireName(robot.sensors[li], li)}"), right_m = the one looking most to the right (index ${ri}, "${sensorWireName(robot.sensors[ri], ri)}"), in metres; no valid reading = that sensor's max range.`,
    '- e = clamp(left_m - right_m - SETPOINT, +-ERROR_CLIP)          (positive = robot is right of centre = turn left)',
    '- integral = clamp(integral + e x dt, +-INTEGRAL_LIMIT)',
    '- d_raw = (e - previous_e) / dt  (0 on the first step)',
    '- alpha = 1 - (1 - D_ALPHA)^(dt / 0.05);  d_filt = alpha x d_raw + (1 - alpha) x d_filt',
    '- turn = clamp(KP x e + KI x integral + KD x d_filt, +-V_MAX)',
    '- base = V_MIN + SPEED_FRAC x (V_MAX - V_MIN)',
    '- v_left = clamp(base - turn, +-V_MAX), v_right = clamp(base + turn, +-V_MAX)   (m/s, the targets of the wheel loops)',
    `Numbers: KP ${g6(p.kp)} (m/s) per m, KI ${g6(p.ki)} (m/s) per (m*s), KD ${g6(p.kd)} (m/s) per (m/s), ERROR_CLIP ${g6(p.error_clip_mm / 1000)} m, INTEGRAL_LIMIT ${g6(p.integral_limit)} m*s, D_ALPHA ${g6(p.d_filter_alpha)} (no unit, per 50 ms), SPEED_FRAC ${g6(p.speed_frac)} (0..1), V_MAX ${g6(vmax)} m/s, V_MIN ${g6(vmin)} m/s, SETPOINT ${g6(-((p.offset_mm || 0) / 1000) * (1 / Math.max(0.17, Math.abs(Math.sin(angles[li] * Math.PI / 180))) + 1 / Math.max(0.17, Math.abs(Math.sin(angles[ri] * Math.PI / 180)))))} m (0 = centred).`,
    `Cruise speed = ${cruiseSpeed(robot).toFixed(3)} m/s. These steering gains use seconds and metres, so they do not depend on the loop period.`,
    '');

  w('## E. Current numbers (every value with its unit)');
  for (const r of numberTable(robot)) w(`- ${r.name} (${r.path}): ${g6(r.value)} ${r.unit}`);
  w('');

  w('## F. Converting to the student\'s units (do this before using any gain)',
    `Symbols: P = wheel loop period in s (${P / 1000}), mpt = metres per tick (${g6(mpt)} m), D = wheel diameter in m (${robot.wheels.diameter_mm / 1000}), V_MAX = top speed (${g6(vmax)} m/s).`,
    '- Speed: m/s = tp x mpt / P;  tp = m/s x P / mpt;  RPM (wheel) = m/s / (pi x D) x 60;  fraction of top speed = m/s / V_MAX.',
    '- Wheel gains when the student\'s speed error is in another unit U: let k = how many U one tp is (m/s: k = mpt / P; fraction of top speed: k = mpt / P / V_MAX; RPM: k = mpt / P / (pi x D) x 60).',
    '  KP_U = KP / k, KI_U = KI / k, KD_U = KD / k, FF_GAIN_U = FF_GAIN / k, FF_STATIC unchanged.',
    '  If their output is 0..1 instead of PWM counts 0..255: divide every gain AND FF_STATIC by 255 as well.',
    '- Integral as e x dt (seconds) instead of a sum of errors: KI_seconds = KI / P. Derivative as de/dt instead of (e - previous_e): KD_seconds = KD x P.',
    '- Another wheel loop period B (r = B / A, A = our period): KP_B = KP_A / r, KI_B = KI_A, KD_B = KD_A / r^2, FF_GAIN_B = FF_GAIN_A / r, FF_STATIC unchanged.',
    '- Ticks per revolution different (other decoding): tp scales by new_ticks / old_ticks, so divide KP, KI, KD, FF_GAIN by the same factor.',
    '- No feedforward in the student\'s code (no start PWM / ff): add it as in section C. Without it the PID gains must be much bigger and the Tuner\'s gains will not work.',
    '- Steering gains: units are m, s and m/s. If the student measures distances in mm or cm, or wheel speeds in another unit, convert the error and the output the same way.',
    '');

  w('## G. Robot file for Robot Tuner (give it back as a file named robot.robot.txt)',
    'The student loads this file in Robot Tuner (Robot builder -> Load / Save -> Load robot file). Below is their current file.',
    'Rules:',
    `- Keep the format exactly: the first line "${FILE_HEADER}", then one "key value" per line; everything after # is a comment.`,
    '- Fill in every value you can read from the student\'s code (pins, motor and encoder signs, ticks per wheel turn, wheel size, gains, loop periods, sensor types, I2C addresses ...), in the unit written in its comment. Convert units if their code uses other ones.',
    '- A value that is already set: keep it, unless the student\'s code clearly says otherwise (then mention it in your list of changes).',
    '- A value you cannot find: leave N/A. Never guess one. Ask the student for the ones marked * (needed to simulate) that stay N/A.',
    '- Keep the keys. Only if the robot has another number of distance sensors, or another encoder / driver / board type than below: change sensor_count or the type, and add or drop the matching lines in the same pattern (sensor<N>_..., pin_...). Text values use _ instead of spaces.',
    '',
    '```',
    ...robotFileTemplate.trimEnd().split('\n'),
    '```',
    '');

  w(
    '## Check list for the student (tell them to do this after uploading)',
    '1. Upload with the Arduino IDE, open Robot Tuner -> Robot builder -> "I have my own code" -> CONNECT.',
    '2. CAL TICKS: turn each wheel one turn by hand forward; both counts must be positive and match the ticks per revolution.',
    '3. Wheels in the air: CAL MAXSPEED, CAL FF. Both wheels must turn forward.',
    '4. If something fails, paste the exact error or serial output back here.');
  return T.join('\n') + '\n';
}
