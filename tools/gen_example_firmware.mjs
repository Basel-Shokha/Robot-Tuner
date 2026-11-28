// node tools/gen_example_firmware.mjs - rewrites the example robot's sketch (the default robot of the site =
// vendor/simulation-to-real-life/robot_hardware.txt: ESP32, DRV8833, quadrature encoders, 3x VL53L1X) into
// firmware/robot_tuner_min/robot_tuner_min.ino and web/firmware/robot_tuner_min.ino.
// tests/js/firmware.test.mjs fails when they are out of date.
import { writeFileSync } from 'node:fs';
import { exampleRobot } from '../web/js/robotfile.js';
import { generateSketch } from '../web/js/firmware-gen.js';
const r = exampleRobot();
const { code } = generateSketch(r, r.hardware);
for (const f of ['firmware/robot_tuner_min/robot_tuner_min.ino', 'web/firmware/robot_tuner_min.ino']) writeFileSync(new URL('../' + f, import.meta.url), code);
console.log('example sketch written');
