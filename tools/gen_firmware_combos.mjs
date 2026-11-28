// node tools/gen_firmware_combos.mjs <out_dir> [--board uno|mega|esp32] [--limit n]
// Writes one sketch folder per hardware combination the robot builder can generate (pins chosen by suggestPins,
// which must find valid pins), plus <out_dir>/combos.txt with "<fqbn> <folder>" per line, for
// tools/compile_firmware.sh (arduino-cli) in CI.
//
// Combinations: every board x encoder variant x driver variant (sensor set rotating), plus every board x every
// sensor set. The generated code is assembled from independent sections (encoder / driver / sensors), so this
// covers every section on every board and every section next to every other section of the same board.
import { mkdirSync, writeFileSync, rmSync } from 'node:fs';
import { join } from 'node:path';
import { defaultRobot, makeSensor } from '../web/js/store.js';
import { generateEsp32Install } from '../web/js/firmware-esp32.js';
import { BOARDS, generateSketch, suggestPins, validateHardware, hasErrors, defaultHardware } from '../web/js/firmware-gen.js';

export const ENCODER_VARIANTS = [
  { id: 'quad', encoder: { type: 'quadrature', avr_mode: 'x1', ppr: 7, gear: 296.125 } },
  { id: 'quadx4', encoder: { type: 'quadrature', avr_mode: 'x4', ppr: 11, gear: 30 } },
  { id: 'quadlow', encoder: { type: 'quadrature', avr_mode: 'x1', ppr: 3, gear: 1 } },           // 3 or 12 ticks/rev: period mode
  { id: 'single', encoder: { type: 'single', ppr: 20, gear: 1 } },                              // LM393 + 20-slot disc
  { id: 'singlehi', encoder: { type: 'single', ppr: 120, gear: 1 } },
  { id: 'as5600mux', encoder: { type: 'as5600', as5600_bus: 'mux', gear: 1 } },
  { id: 'as5600bus', encoder: { type: 'as5600', as5600_bus: 'two_buses', gear: 1 }, boards: ['esp32'] },
  { id: 'noenc', encoder: { type: 'none' } },
];
export const DRIVER_VARIANTS = [
  { id: 'in12brake', driver: { type: 'in1_in2', stop: 'brake' } },
  { id: 'in12coast', driver: { type: 'in1_in2', stop: 'coast' } },
  { id: 'pwmdir', driver: { type: 'pwm_dir' } },
  { id: 'enbrake', driver: { type: 'en_in1_in2', stop: 'brake' } },
  { id: 'encoaststby', driver: { type: 'en_in1_in2', stop: 'coast', stby: 99 } },  // 99 = "give it a pin"
];
const S = (type, name, angle) => makeSensor(type, { name, x_mm: 100, y_mm: 0, angle_deg: angle });
export const SENSOR_SETS = [
  { id: '3l1x', sensors: () => [S('VL53L1X_short', 'right', -45), S('VL53L1X_short', 'front', 0), S('VL53L1X_short', 'left', 45)] },
  { id: '3l0x', sensors: () => [S('VL53L0X', 'right', -90), S('VL53L0X', 'front', 0), S('VL53L0X', 'left', 90)] },
  { id: 'mixtof', sensors: () => [S('VL53L0X', 'right', -60), S('VL53L1X_long', 'front', 0), S('VL53L1X_short', 'left', 60)] },
  { id: '1l0x', sensors: () => [S('VL53L0X', 'front', 0)] },
  { id: '2sonar', sensors: () => [S('HC_SR04', 'right', -90), S('HC_SR04', 'left', 90)] },
  { id: '2sharp', sensors: () => [S('GP2Y0A21', 'right', -90), S('GP2Y0A41', 'left', 90)] },
  { id: 'all5', sensors: () => [S('VL53L1X_short', 'right', -45), S('HC_SR04', 'front', 0), S('GP2Y0A21', 'left', 45), S('VL53L0X', 'rf', -20), S('GP2Y0A41', 'lf', 20)] },
];

export function buildCombo(board, enc, drv, set) {
  const r = defaultRobot();
  r.sensors = set.sensors();
  r.source = {};
  const base = defaultHardware();
  const hw = { ...base, board, encoder: { ...base.encoder, ...enc.encoder }, driver: { ...base.driver, ...drv.driver, stby: drv.driver.stby ? 99 : -1 },
    sensors: r.sensors.map((s, i) => ({ xshut: -1, addr: 0x30 + i, trig: -1, echo: -1, analog: -1 })) };
  // every pin role gets a fresh pin
  for (const s of hw.sensors) if (r.sensors.length > 0) s.xshut = r.sensors.length > 1 ? 99 : -1;
  const { hw: pinned, failed } = suggestPins(r, hw);
  return { robot: r, hw: pinned, failed };
}

export function allCombos() {
  const out = [];
  for (const board of Object.keys(BOARDS)) {
    let k = 0;
    for (const enc of ENCODER_VARIANTS) {
      if (enc.boards && !enc.boards.includes(board)) continue;
      if (board === 'esp32' && enc.id === 'quadx4') continue;  // the ESP32 has no x1 / x4 choice (= 'quad')
      for (const drv of DRIVER_VARIANTS) out.push({ board, enc, drv, set: SENSOR_SETS[k++ % SENSOR_SETS.length] });
    }
    for (const set of SENSOR_SETS) out.push({ board, enc: ENCODER_VARIANTS[0], drv: DRIVER_VARIANTS[0], set });
  }
  return out.map((c) => ({ ...c, name: `${c.board}_${c.enc.id}_${c.drv.id}_${c.set.id}` }));
}

if (import.meta.url === `file://${process.argv[1]}`) {
  const outDir = process.argv[2] || 'build/firmware';
  const bi = process.argv.indexOf('--board'), only = bi > 0 ? process.argv[bi + 1] : null;
  rmSync(outDir, { recursive: true, force: true });
  mkdirSync(outDir, { recursive: true });
  const lines = [], skipped = [];
  const seen = new Set();
  for (const c of allCombos()) {
    if (only && c.board !== only) continue;
    if (seen.has(c.name)) continue;
    seen.add(c.name);
    const { robot, hw, failed } = buildCombo(c.board, c.enc, c.drv, c.set);
    if (failed.length) { skipped.push(`${c.name}: not enough pins for ${failed.join(', ')}`); continue; }
    const v = validateHardware(robot, hw);
    if (hasErrors(v)) { console.error(`${c.name}: suggestPins gave invalid pins:`, v.filter((m) => m.level === 'bad')); process.exitCode = 1; continue; }
    const g = generateSketch(robot, hw);
    const dir = join(outDir, c.name);
    mkdirSync(dir, { recursive: true });
    writeFileSync(join(dir, c.name + '.ino'), g.code);
    lines.push(`${g.fqbn} ${c.name}`);
  }
  if (!only || only === 'esp32') {   // the ESP32 install firmware (one build for every ESP32 robot, settings over serial)
    const g = generateEsp32Install(), name = 'robot_tuner_esp32';
    mkdirSync(join(outDir, name), { recursive: true });
    writeFileSync(join(outDir, name, name + '.ino'), g.code);
    lines.push(`${g.fqbn} ${name}`);
  }
  writeFileSync(join(outDir, 'combos.txt'), lines.join('\n') + '\n');
  console.log(`${lines.length} sketches in ${outDir}`);
  if (skipped.length) console.log('skipped (the board has too few pins for this combination):\n  ' + skipped.join('\n  '));
}
