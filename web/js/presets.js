// presets.js - playground presets that show classic PID behaviours, relative to the robot's own gains
// (so they work for any robot). Each returns patched copies of the robot and the environment.
const clone = (x) => JSON.parse(JSON.stringify(x));
function base(robot, env, start = { angle_deg: 25, offset_mm: 0, random: false }) {
  const r = clone(robot), e = clone(env);
  e.mode = 'endless'; e.start = start; e.noise.global = 1; e.noise.delay_ms = 0; e.noise.mismatch_pct = 0;
  return [r, e];
}
export const PRESETS = {
  good: (robot, env) => base(robot, env),
  p_low: (robot, env) => { const [r, e] = base(robot, env, { angle_deg: 0, offset_mm: 40, random: false }); r.pid.kp *= 0.12; r.pid.kd *= 0.3; r.pid.ki = 0; return [r, e]; },
  p_high: (robot, env) => { const [r, e] = base(robot, env); r.pid.kp *= 8; r.pid.kd *= 0.2; r.pid.ki = 0; return [r, e]; },
  no_d: (robot, env) => { const [r, e] = base(robot, env, { angle_deg: 30, offset_mm: 30, random: false }); r.pid.kp *= 2.5; r.pid.kd = 0; r.pid.ki = 0; return [r, e]; },
  i_windup: (robot, env) => {
    const [r, e] = base(robot, env, { angle_deg: 30, offset_mm: 40, random: false });
    r.pid.ki = Math.max(1.5, r.pid.ki * 80); r.pid.integral_limit = 3; e.noise.mismatch_pct = 4; return [r, e];
  },
  // a (virtual) motor 4x faster than measured at the same speed_frac: the cruise speed scales with v_max
  too_fast: (robot, env) => { const [r, e] = base(robot, env); r.sim_max_speed_mps = Math.min(1, 4 * r.motor.max_speed_mps); e.noise.delay_ms = 150; return [r, e]; },
  saturation: (robot, env) => {
    const [r, e] = base(robot, env, { angle_deg: 35, offset_mm: 0, random: false });
    r.pid.speed_frac = 1; e.width_mm = Math.max(e.width_mm, 200); return [r, e];
  },
  // the same gains side by side at speed_frac 1.0 (cruise = v_max: no room left to turn) and 0.75 (the default)
  clip_compare: (robot, env) => {
    const [r, e] = base(robot, env, { angle_deg: 35, offset_mm: 0, random: false });
    e.width_mm = Math.max(e.width_mm, 200); return [r, e, { param: 'speed', values: [1, 0.75] }];
  },
  mismatch: (robot, env) => { const [r, e] = base(robot, env, { angle_deg: 0, offset_mm: 0, random: false }); r.pid.ki = 0; e.noise.mismatch_pct = 8; return [r, e]; },
};
export const PRESET_IDS = Object.keys(PRESETS);
