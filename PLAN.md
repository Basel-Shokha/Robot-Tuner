# Robot Tuner — plan

A static website (GitHub Pages, no backend, works offline once loaded) that helps students make a
differential-drive robot drive parallel and centred between two walls with PID, and test maze-solving
algorithms, using the physics of `Basel-Shokha/Simulation-To-Real-Life` compiled to WebAssembly.

**Scope:** differential drive only — 2 driven wheels, or 4-wheel skid-steer with left/right pairs driven together;
wheel encoders; 1–5 distance sensors. **Out of scope:** mecanum, omni, car-like (Ackermann) steering, robots
without encoders, NN training (later, GitHub Actions).

## Architecture

```
vendor/simulation-to-real-life/   verbatim copy of the upstream sources (commit in SOURCE_COMMIT): PID/, NNMap/,
                                  tools/gen_hardware.py, robot_hardware_builder.html, firmware/robot_fw_run25_nnmap/
                                  (+ the 4 shared firmware headers PID/Robot.hpp includes from firmware/robot_fw/)
sim/                              Robot Tuner's C++ engine
  rt_rng.hpp                      portable copies of libstdc++'s mt19937 distributions (same numbers everywhere)
  rt_engine.hpp                   upstream Robot::Step / PidBrain / EvaluateRobot generalised to 1–5 sensors,
                                  2- or 4-wheel footprint, per-sensor models, external (JS) controller mode.
                                  It #includes the UPSTREAM firmware headers unchanged (wheel_pid.h,
                                  emergency.h, steer_smooth.h) and the upstream StraightGuide/MotorModel ideas.
  rt_api.cpp                      C API exported to JS (instances, walls, step, evaluate, training helpers)
  build_wasm.sh                   emcc -> web/wasm/rt_sim.{js,wasm} (committed, so Pages needs no build)
tests/
  native/equiv_upstream.cpp       rt engine (3 sensors) vs the UNMODIFIED upstream PID/Robot.hpp: bit-identical runs
  native/golden.cpp               native build of the engine writes golden trajectories (tests/golden/*.json)
  wasm_equiv.test.mjs             the WASM build must reproduce the native golden files bit for bit
  js/*.test.mjs                   parsers, maze builder, i18n completeness, robot JSON round trip (node --test)
web/                              the site (plain ES modules, no framework, no build step)
  index.html                      home + scope statement
  builder.html                    1. robot builder (draw + edit, sensor library, JSON / robot_hardware.txt)
  measure.html                    2. guided measurement over USB (Web Serial) + manual mode
  corridor.html                   3. corridor simulator (endless corridor, wall errors, track editor, noise)
  train.html                      4. PID training (grid search + GA in Web Workers), export C + config.h
  playground.html                 5. live PID playground, presets, sweep (ghost robots)
  troubleshoot.html               5b. "on the real robot it does X" -> "try Y" -> preset
  tutorial.html                   6. PID tutorial: structure + demos, text = placeholders for Basel
  maze.html                       7. maze runner sandbox
  js/                             i18n.js, store.js (robot JSON, localStorage), engine.js (WASM wrapper),
                                  draw.js, plot.js, corridor.js, track.js, train-worker.js, serial.js, maze/*.js
  i18n/en.json he.json ar.json    ALL user-visible text; he/ar hold the same keys (empty = falls back to English);
                                  <html dir> switches to rtl for he/ar
  sw.js                           service worker: caches every asset -> offline
firmware/robot_tuner_min/         minimal Arduino sketch answering the same CAL / STATUS commands
README.md (students)  DEVELOPERS.md (developers)
```

### Data model
One robot JSON (`robot-tuner/robot@1`): body, wheels (2wd / 4wd-skid), caster, motor + wheel-PID numbers,
1–5 sensors (type from the library + every value overridable), timing (loop period, reading age), and a
`source` tag per measured value (MEASURED / DATASHEET / TYPED / DEFAULT). The builder, the measurement wizard,
the simulator and the trainer all read/write this one object (localStorage + file export/import). It also
exports `robot_hardware.txt` in the upstream format (explicit `tof_pose_*`), so `tools/gen_hardware.py` works.

### Engine (C++ -> WASM)
* Physics = upstream `Robot::Step`: ToF cone ray casting with noise/dropout/glitch filter, reading age (pose
  history), wheel model `pid` (upstream `firmware/robot_fw/wheel_pid.h` on the DC-motor model, encoder
  quantisation), slip, per-run hardware bias, steering smoother, PID controller (`PidBrain::Control`).
* Generalisation: N = 1..5 sensors with their own cone/range/noise/dropout/period; the PID's left/right sensors
  are the most CCW / most CW ones (upstream rule); with 1 side sensor it follows that wall at a set distance.
  4WD skid-steer = same kinematics with the measured effective track width, 4 wheel rectangles for collisions.
* Determinism: the engine uses `rt_rng.hpp` (bit-exact copies of libstdc++'s `uniform_real_distribution<float>`
  and `normal_distribution<float>`) so g++ native, clang/libc++ and WASM produce the same numbers; float math
  goes through one switch (`RT_PORTABLE_MATH`) so the WASM build and the native test build agree exactly.
* External controller mode: JS (maze algorithms) supplies wheel targets each tick; the same sensor/encoder/
  motor models apply.

### Equivalence ("keep sim results identical to the C++ build")
1. `tests/native/equiv_upstream.cpp`: for several configs (noise on/off, latency, wheel models, wall jitter), the
   upstream `Robot` + `PidBrain` and the rt engine are stepped side by side; pose, readings and wheel commands
   must be bit-identical every tick (`make -C tests test-native`).
2. `tests/native/golden.cpp` writes trajectories + training scores from the native build; `tests/wasm_equiv.test.mjs`
   loads `web/wasm/rt_sim.wasm` in Node and requires identical bits. CI runs both (`.github/workflows/test.yml`).

## Milestones (each one committed)
- **M0** PLAN.md, vendor copy with source commit, skeleton READMEs.
- **M1** C++ engine + native equivalence vs upstream + WASM build + golden/WASM test.
- **M2** Site shell: i18n (en/he/ar, RTL), nav, robot store, drawing + plot helpers, service worker, scope banner.
- **M3** Robot builder (live drawing, sensor library, JSON + robot_hardware.txt export/import).
- **M4** Corridor simulator (endless scrolling corridor, wall error types with sliders/toggles, track editor,
  start pose, noise sliders) + Playground (live gains, plots, presets, sweep/ghosts) + Troubleshooting page.
- **M5** PID training in Web Workers (grid -> GA), robustness report 0x/1x/2x, export C function + config.h.
- **M6** Guided measurement over Web Serial (CAL TICKS/MAXSPEED/FF/STEP/DRIVE/SPIN/TOF, LOAD SENSORS + STATUS
  for loop period and reading age), sanity checks, manual mode, minimal Arduino sketch.
- **M7** PID tutorial page (structure, 4 interactive demos, i18n placeholder slots).
- **M8** Maze sandbox: thick/thin grid mazes built wall by wall with accumulating construction error, random
  solvable mazes + drawing, pluggable JS algorithms (wall follower, flood fill, DFS + mapping), belief map vs real
  maze, metrics (exit, time, distance, crashes, map error), batch robustness runs.
- **M9** READMEs (students / developers), tests in CI, offline check, polish.
