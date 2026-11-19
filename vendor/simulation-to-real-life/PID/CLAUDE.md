# CLAUDE.md (PID variant)

This folder is the PID copy of `../NN`. Everything below the line describes the shared simulator; the
differences are listed first.

## PID differences from NN

- `PidController.hpp` (`PidBrain`) replaces `Brain.hpp` (`NeuralBrain`) with the same interface
  (`hw`, `Mutate`, `Save`, `Load`, constructor from `SimConfig`), plus `Control()` and `ResetState()`.
  `err = d[left] - d[right]` from the two side ToF sensors (left = largest mount angle), clipped to
  `pid_error_clip_m`; turn = Kp*err + Ki*integral + Kd*filtered derivative; `v_r = base + turn`,
  `v_l = base - turn`, base slowed by `pid_slowdown_frac` in hard turns. Encoders are not inputs.
- Trained parameters: `kp`, `ki`, `kd` (and `base_speed_frac` when `pid_train_base_speed 1`).
  `Mutate()` scales each gain by `exp(N(0, 0.25))`, at least one per call. Model file:
  `best_pid_model.txt` (key/value text, starts with `controller pid`; NN files are rejected).
- `pid_params.txt` holds the untrained shaping knobs, parsed by `LoadConfig` after `error_params.txt`.
- `Robot.hpp`: NAVIGATING calls `brain.Control(dists, SIMULATION_DT, v_l, v_r)`; PID state is reset in
  `Robot::Reset` and when a scan hands control back.
- `export_gains.cpp` replaces `export_weights.cpp`: writes `pid_gains.h` (C99 gains + `pid_control()`
  with the simulator's exact arithmetic). `tof_experiment.cpp` and NN model files are not copied.
- `train.cpp`, `benchmark.cpp`, `visualizer.cpp`: the reward and benchmark logic are unchanged
  (so PID and NN are compared on equal terms); only the brain type, model file name and printed gains differ.

---

# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project overview

Sim-to-Real Autonomous 3-ToF Differential Drive Suite: a C++17 simulator that evolves a small neural-network
"brain" (via a hillclimbing/elitism genetic algorithm, not backprop) to drive a differential-drive robot
using 3 time-of-flight distance sensors, then exports the trained weights as a standalone C header for
deployment on real embedded hardware. Everything is header-only except for the four `.cpp` entry points.

## Build

Windows/MinGW with `mingw32-make` (or `make` on Linux), requires `libxlsxwriter` (charts) and `raylib`
(visualizer only). See `BUILD_INSTRUCTIONS.txt` for installing `libxlsxwriter` from source/MSYS2.

```
mingw32-make all                                    # builds train, benchmark, export_weights, visualizer
mingw32-make all XLSX_INC="-I C:/libxlsxwriter/include" XLSX_LIB="-L C:/libxlsxwriter/lib -lxlsxwriter -lz"
mingw32-make train        # build + run train.exe
mingw32-make benchmark    # build + run benchmark.exe
mingw32-make export       # build + run export_weights.exe -> weights.h (from best_robot_model.txt)
mingw32-make visual       # build + run visualizer.exe (raylib GUI)
mingw32-make clean
```

`XLSX_INC`/`XLSX_LIB` and (for visual) raylib paths default to `C:/libxlsxwriter-install` and
`C:/raylib/raylib/src`; override them if installed elsewhere. Run binaries from the repo root: every config,
model, track and output path is relative to the working directory. There is no test suite — validation is done
by running `train`/`benchmark` and inspecting fitness curves / crash rates.

## Architecture

**Data flow across the 4 binaries**, all sharing the same header-only simulation core:

1. **`train.cpp`** — genetic-algorithm trainer. Population of `NeuralBrain` candidates evaluated in parallel
   (OpenMP) each generation via `EvaluateRobot()`, which runs a full episode and returns a scalar fitness
   (exploration coverage, speed tracking, corridor-centering, scan-and-turn correctness, crash/stall
   penalties). Strict elitism: each generation, the single best-ever brain (`global_best`) reseeds the whole
   population with mutated copies (`NeuralBrain::Mutate`). Checkpoints to `best_robot_model.txt` whenever a
   new all-time best is found, so re-running `train` resumes from there. Runs until Ctrl+C (`SIGINT` sets
   `g_stop_requested`), writing `training_log.csv` and an xlsx convergence chart
   (`Graphs/training_graphs/generation_to_fitness/<track>_<N>.xlsx`) every 25 generations and on exit.
   Optional CLI (all defaults preserve the historical behaviour): `--seed N` (mixed into every random
   stream; without it the trainer is fully deterministic and repeat runs return an identical brain),
   `--gens N`, `--out PATH`, `--resume PATH|none`, `--signal-model 0|1`, `--rays N`, `--quiet`
   (suppress `training_log.csv` and the xlsx, for batch experiments).
2. **`benchmark.cpp`** — loads a saved brain and sweeps a sensor/actuator noise multiplier against the
   trained track, measuring crash rate and completion time, exporting to
   `Graphs/benchmark_graphs/noise_to_crash_rate/`. This is the "sim-to-real robustness" check: how far can
   noise be pushed above training-time levels before the policy breaks.
3. **`visualizer.cpp`** — raylib GUI that replays a brain on a track in real time and exports per-tick motor
   telemetry to `Graphs/visual_graphs/time_to_motor_velocity/`.
4. **`tof_experiment.cpp`** — measurement harness for comparing the two ToF sensor models. Three
   subcommands, all writing CSV to stdout: `cost` (wall-clock per generation-equivalent workload,
   single-threaded, at a given model and ray count), `eval` (crash/completion rate for a saved brain
   under a chosen sensor model — used to test brains trained under one model against another), and
   `conservatism` (walks a real trajectory and compares both models' *noiseless* readings at
   identical poses). Build with `mingw32-make tof-experiment`; no xlsxwriter dependency.
5. **`export_weights.cpp`** — converts `best_robot_model.txt` into a dependency-free, static, C99-compatible
   `weights.h` (network weights + hardware/sensor-geometry `#define`s) meant to be dropped onto real
   microcontroller firmware — the actual "sim to real" handoff artifact.

**Header responsibilities:**
- `Types.hpp` — `Vec2`, `Wall` (line-segment obstacle), `HardwareConfig`.
- `SimConfig.hpp` — `SimConfig`/`SensorMount`; `LoadConfig()` parses `sim_config.txt` (geometry/track) and
  `error_params.txt` (noise/slip parameters) as simple whitespace key-value text files. Unknown/missing keys
  silently fall back to defaults (warns to stdout).
- `Raycast.hpp` — ToF sensor model (VL53L1X). Casts a cone of `tof_rays` rays (default 3) spanning
  `TOF_CONE_HALF_ANGLE` per sensor against `Wall`s. Two selectable models, chosen by
  `tof_signal_model` in `error_params.txt`:
  - **min() (default, cheap)** — closest hit across the cone, plus Gaussian noise and random dropout.
    Not physically how the sensor works, but one raycast per ray and no normals.
  - **signal model** — `SampleToFSignalModel`: per-ray returned signal
    `~ reflectance * cos^k(incidence) * offaxis_weight / d^2`, rays grouped into histogram peaks,
    strongest peak reported at its signal-weighted centroid, reading dropped below the ULD signal
    threshold, noise growing as `1/sqrt(signal)` for weak returns. Coefficients are named and
    runtime-configurable, each marked DATASHEET / DERIVED / ASSUMED.

  `TOF_CONE_HALF_ANGLE` is the **horizontal** half-angle, derived from the datasheet's **diagonal**
  FoV (27° for the default 16x16 SPAD ROI). Re-derive it if the firmware ever programs a narrower
  ROI. Changing anything here invalidates every saved model file.
- `Track.hpp` — track generation. `"benchmark"` is a hardcoded double-loop track
  (`GenerateBenchmarkTrack`); otherwise loads `tracks/<name>.track`, which is either a "freeform" file
  (explicit `spawn`/`wall` line-segment commands) or a turn-sequence corridor format (`corridor_width`,
  `segment_length`, `start_heading`, then a `segments:` list of turn angles building a centerline that gets
  offset into left/right walls). `PerturbTrack()` adds per-generation Gaussian wall jitter during training
  (domain randomization).
- `Brain.hpp` — `NeuralBrain`: fixed-topology 5-8-2 tanh MLP (inputs: 3 normalized ToF distances + 2 noisy
  normalized wheel velocities; outputs: left/right wheel velocity commands). `Mutate()` does per-weight
  Gaussian perturbation with 20% probability per weight (this *is* the only learning mechanism — no
  gradient descent). `Save`/`Load` use a simple whitespace-delimited text format (`best_robot_model.txt`
  et al.). The header line stores `right_angle_deg track_width`; `Load` discards the angle, so sensor angles
  always come from `sim_config.txt`. Changing the 5-8-2 topology breaks every saved model file and
  `export_weights.cpp`.
- `Robot.hpp` — physics/behavior integration step (`SIMULATION_DT = 0.05s`). Implements a hybrid
  finite-state controller layered on top of the learned network: `NAVIGATING` (network in control) can
  trigger a hardcoded `SCAN_TURN_LEFT` -> `SCAN_TURN_RIGHT` -> `SCAN_ALIGN_BEST` sequence when the center
  sensor reads below `emergency_stop_threshold`, comparing left/right scan readings and re-aligning toward
  the more open side before returning control to the network. Also applies wheel-slip-under-acceleration and
  asymmetric ToF glitch rejection (only sudden *increases* in reading beyond `tof_glitch_threshold_m` are
  rejected; decreases are always accepted).
- `GraphExportHelper.hpp` — shared xlsxwriter output helpers; auto-increments a run index per track name so
  repeated runs don't clobber prior `.xlsx` outputs in `Graphs/<category>/<subcategory>/`.

**Config files** (repo root, plain text, not JSON/YAML): `sim_config.txt` (geometry + which track to load),
`error_params.txt` (sensor/actuator noise). Both are read by `SimConfig::LoadConfig` and apply to all four
binaries identically, so changing noise/track parameters affects train/benchmark/visualize consistently.

**Browser tools** (standalone, no build step, open directly in a browser): `track_builder.html` (draws
and exports freeform `.track` files, which must be moved into `tracks/`) and `robot_hardware_builder.html`
(visual hardware/sensor-geometry editor that exports `sim_config.txt`). The exported `sim_config.txt` must
stay in sync with the keys `LoadConfig` parses.

**Saved model files** at repo root (`best_robot_model.txt`, `champion_*.txt`, `master_brain.txt`, etc.) are
checkpoints from different training runs/tracks — treat them as generated artifacts, not source.

## Straight_Wall_Project differences

This copy only trains on straight tracks. The goal is to start rotated up to 80° (possibly off-centre) and
then drive the whole track parallel to the walls and centred between them, without being thrown off by wall errors.
- `StraightGuide.hpp` (built per track in `LoadNamedTrack`) gives ground truth to the reward only:
  a global track axis (length-weighted axial mean of wall directions) and a centreline taken from the median
  lateral position of each wall over ±0.6 m. Slanted segments, bumps, gaps and per-episode jitter barely move either.
- `train.cpp` `EvaluateRobot`: each metre along the axis is worth `100 * (1 + parallel_weight*p + center_weight*c)`,
  where p and c are Gaussian scores of the heading error and centre error. Backward metres are charged at the full rate.
  The old sensor-asymmetry centering term is gone. Episodes 1-3 randomize the spawn heading and lateral offset
  (`spawn_heading_random_deg`, `spawn_lateral_random_frac`). All of these keys are in `error_params.txt`.
- `tools/gen_straight_tracks.py` regenerates the seven `tracks/straight_*.track` files (long tracks; varied widths, spawn headings
  and mixed-width sections).

## Real robot hardware (Nov 2026)

- Hardware values come from `../robot_hardware.txt` (the single source). Do not edit the generated block of
  `sim_config.txt` by hand: run `python tools/gen_hardware.py` from the repo root.
- The robot origin is the drive-axle midpoint (rotation centre); the outline is `chassis_front_m` /
  `chassis_rear_m` around it; each ToF ray starts at its sensor's pose. Sensor index 0 = right (-45 deg),
  1 = front, 2 = left (+45 deg), angles CCW, +y = left.
- `error_params.txt` has the real-robot timing models: `encoder_model`, `motor_tau_s`, `motor_max_accel_mps2`,
  `tof_delay_steps`, `spawn_at_body_center`. Their defaults in `SimConfig.hpp` reproduce the old behaviour.
- `firmware/robot_fw/controllers.h` is a 1:1 port of the controllers; `tools/equiv/run_equiv.sh` must keep
  passing after any change to Robot.hpp / Brain.hpp / PidController.hpp / LineFitController.hpp.
- `wheel_model pid` (MotorModel.hpp) runs the firmware wheel PID (`../firmware/robot_fw/wheel_pid.h`, included, not copied) on a DC-motor model; keep MotorModel.hpp identical in the three folders.
- See `docs/hardware_audit.md` for the audit and benchmark.
