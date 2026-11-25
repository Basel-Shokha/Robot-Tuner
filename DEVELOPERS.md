# Robot Tuner — developer notes

Static site in `web/` (plain ES modules, no framework, no build step for the JS), a C++ engine in `sim/` compiled
to WebAssembly (`web/wasm/`, committed so Pages needs no build), tests in `tests/`. Plan and milestones: `PLAN.md`.

## Layout

```
vendor/simulation-to-real-life/  upstream sources, unmodified (see SOURCE_COMMIT)
  PID/ robot_hardware.txt                  (the robot firmware itself is deliberately NOT copied: it holds WiFi
                                           credentials; the Measure page follows its CAL command protocol)
  firmware/robot_fw/{wheel_pid,emergency,steer_smooth,reflex}.h   included by PID/Robot.hpp (needed to build it)
  tracks/                                  four upstream tracks used by the tests
sim/rt_rng.hpp        portable RNG distributions (bit-exact copies of libstdc++'s) + float math switch
sim/rt_engine.hpp     the engine: port of upstream Robot::Step, PidBrain, DriveSim, PerturbTrack, RandomSpawn,
                      StraightGuide (polyline), EvaluateRobot; generalised to 1-5 sensors, 4WD, external control
sim/rt_api.cpp        C API (instances, config text, walls, step, state, rays, footprint, evaluate)
sim/build_wasm.sh     em++ -> web/wasm/rt_sim.{mjs,wasm}
tests/native/equiv_upstream.cpp   engine vs UNMODIFIED upstream Robot.hpp: bit-identical every tick
tests/native/golden.cpp           native reference runs through the C API -> tests/golden/
tests/wasm_equiv.test.mjs         the WASM module must reproduce tests/golden/ bit for bit
tests/js/*.test.mjs               parsers, maze builder, i18n completeness, robot JSON round trip, config keys
firmware/robot_tuner_min/         minimal Arduino sketch answering the measurement commands
web/js/engine.js      JS wrapper of the WASM API (layout of rt_state() in `S`)
web/js/store.js       robot JSON (schema robot-tuner/robot@1), sensor library, robot -> engine config,
                      robot_hardware.txt export/import
web/js/corridor.js    endless corridor generator (wall errors), tracks from centre lines, .track import, start pose
web/js/simview.js     Scene / Runner / drawScene (shared by corridor, playground)
web/js/train-*.js     training (worker pool, grid + GA), export.js (C function, firmware defines)
web/js/serial.js, measure-steps.js (pure parsers + sanity checks), measure.js
web/js/maze/          maze.js (generation + wall-by-wall construction), mazekit.js (odometry, mapper, motion,
                      CellRobot), algorithms.js (examples + custom template), maze-page.js
web/i18n/             en.json (source of truth), he.json, ar.json
web/sw.js             offline cache (bump VERSION when releasing)
```

## Build and test

```sh
make -C tests test-native        # needs g++ (C++17)
make -C tests test-wasm          # needs node >= 20; uses the committed web/wasm build
./sim/build_wasm.sh              # needs Emscripten (em++) on PATH: source <emsdk>/emsdk_env.sh
make -C tests golden             # regenerate tests/golden after an INTENDED engine change
```

CI (`.github/workflows/test.yml`) runs all of it and also rebuilds the WASM from source. `pages.yml` publishes
`web/` (Settings -> Pages -> Source: GitHub Actions).

## "Identical to the C++ build" — how it is guaranteed

* **Upstream equivalence.** `equiv_upstream.cpp` compiles the upstream `PID/Robot.hpp`, `PidController.hpp`,
  `MotorModel.hpp`, `Track.hpp`, `Spawn.hpp` *unchanged* next to the engine and steps both on upstream tracks for
  12 configurations (both wheel models, latency + jitter, delay steps, emergency manoeuvre, wall jitter,
  random spawns, culling on/off, 7 rays). Pose, wheel commands, encoders and every sensor reading must be
  bit-identical every tick. Built with g++/libstdc++ and `RT_PORTABLE_MATH=0` = the upstream arithmetic.
* **Portable randomness.** `std::normal_distribution` / `uniform_real_distribution` differ between libstdc++ (g++)
  and libc++ (Emscripten). `rt_rng.hpp` re-implements libstdc++'s algorithms exactly (checked against the real ones
  in the test), so the same seed gives the same numbers in the browser.
* **Portable float math.** glibc and Emscripten's musl round `sinf/cosf/expf/...` differently in rare cases (tried:
  with upstream float math the WASM run diverges). With `RT_PORTABLE_MATH=1` (the default; used by the WASM build
  and by the native reference build) these are evaluated in double and rounded to float, so both platforms agree.
  `golden.cpp` (native) and `wasm_equiv.test.mjs` (WASM) run the same scenarios through the same C API and must give
  the same per-tick hashes. Compile with `-ffp-contract=off` (no FMA contraction) everywhere.

Differences of the engine from upstream (all off in the equivalence configurations, or extensions):
not ported: `fixed_clock`, `reflex`, `slow`, `speed_law`, `exp_*` experiments. Added: per-sensor models
(ToF cone min / IR beam / ultrasonic with max incidence), 1-5 sensors, 4WD footprint, `ctrl_track_width_m`,
fixed wheel mismatch, one-wall PID mode, external controller mode, wall culling (provably result-neutral).
Training uses the same reward and seeds as upstream `train.cpp` (`Evaluate`), with a grid search before the GA.

## Robot JSON

`schema: "robot-tuner/robot@1"`; units mm, deg, ms, m/s; frame = upstream (origin at the drive-axle midpoint,
+x forward, +y left, angles CCW). `source` records where each value came from (MEASURED / TYPED / IMPORTED ...).
`migrate()` fills missing fields from the defaults, so hand-written or older files load. Defaults = the upstream
real robot (`vendor/.../robot_hardware.txt`).

## Maze algorithms

Interface: `create(ctx, kit) -> { step(obs) -> {left, right} }` with `obs = {t, dt, dists, enc, encDist}` — only what a
real robot has. `kit.CellRobot` provides odometry, the front-sensor "look around" mapping at cell centres,
side-wall centring while driving, and turn/drive motion with acceleration ramps; an algorithm then only decides the
next neighbour cell. The page draws `algo.map` / `algo.pose` / `algo.visited` if present. To add a C++/WASM
algorithm later, expose a `step` with the same inputs from a second WASM module.

## Translating

Edit `web/i18n/he.json` / `ar.json` (empty string = English fallback); run `node tools/i18n_sync.mjs` after adding
keys to `en.json` (the test fails otherwise). Keep `{placeholders}` and markup. RTL works through `<html dir>` and
CSS logical properties; canvases stay left-to-right. The tutorial text slots are `tut.s1.body` … `tut.s6.body`.

## Measurement protocol

The Measure page sends the commands of the upstream firmware `firmware/robot_fw_run25_nnmap` (not copied here; `CAL TICKS`/`END`, `CAL MAXSPEED`,
`CAL FF`, `CAL STEP`, `CAL DRIVE`/`DIST`, `CAL SPIN`/`TURNS`, `CAL TOF n true_mm`, `LOAD SENSORS` + `STATUS` +
`LOAD FINAL`) and parses the result lines in `measure-steps.js` (unit-tested with real lines). The minimal sketch
prints the same lines. Photos/GIFs for each step go to `web/img/measure/<step>.gif` (placeholders shown now).

## Firmware generator

`web/js/firmware-gen.js` builds `robot_tuner_min.ino` from `robot.hardware` (board, driver, encoders, sensor pins;
edited on the builder's Firmware tab). The calibration code (`CORE`) prints exactly the lines `measure-steps.js`
parses: change them together. After changing the generator:

- `node tools/gen_example_firmware.mjs` rewrites the example robot's sketch (`firmware/` and `web/firmware/`; a test
  checks they are current),
- `make firmware` generates every combination (`tools/gen_firmware_combos.mjs`) and compiles it with arduino-cli
  (cores `arduino:avr`, `esp32:esp32`; libraries `VL53L0X`, `VL53L1X`, `PinChangeInterrupt`). CI runs it on the latest
  cores and on esp32 core 3.3.

## Constant-speed PID

The site runs the PID with a constant cruise speed: `sim/rt_engine.hpp` `Pid::Control` with `pid_speed_mode 1`
(also read as upstream's `cs_mode`, `cs_speed_frac`, `cs_v_min_mps`). It is upstream `PID/PidController.hpp` with
`const_speed` (Simulation-To-Real-Life `f27805a`, vendored at `a929bcb`): turn clamped to +-v_max,
base = v_min + speed_frac (v_max - v_min), wheels base -+ turn clamped to +-v_max, no slowdown. Defaults are upstream's
(speed_frac 0.75, v_min 0.013 m/s). Tests: `tests/native/equiv_upstream.cpp` runs four constant-speed cases against
the vendored upstream bit for bit, `tests/native/pid_const.cpp` checks the formula, the golden scenarios
`const_speed*` check native == WASM, and `tests/js/pid.test.mjs` checks the exported C against the engine (g++).
