// node tools/gen_example_robot.mjs - writes robots/basel_nicole.robot.txt and .robot.json: our robot (Simulation-To-Real-Life
// robot_hardware.txt), every value set. Load it in Robot builder -> File -> Load robot file.
import { writeFileSync } from 'node:fs';
import { exampleRobot, robotToFile } from '../web/js/robotfile.js';
const r = exampleRobot();
writeFileSync(new URL('../robots/basel_nicole.robot.txt', import.meta.url), robotToFile(r));
writeFileSync(new URL('../robots/basel_nicole.robot.json', import.meta.url), JSON.stringify(r, null, 2) + '\n');
console.log('robots/basel_nicole.robot.txt and .robot.json written');
