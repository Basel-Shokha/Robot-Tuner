# Robot Tuner

**Make your robot drive straight down the middle of a corridor — and then solve mazes.**

Robot Tuner is a website for students who build a small robot that must drive **parallel to and centred
between two walls** using a **PID controller**. Describe your robot, measure it over USB, tune and train the PID
gains in a simulator, see *why* each gain behaves as it does, and then test maze-solving algorithms on the same
simulated robot.

It runs entirely in your browser (no account, no server) and keeps working offline once it has loaded.
Open it at the GitHub Pages address of this repository, or locally (see below).

## Which robots?

| Supported | Not supported |
|---|---|
| Differential drive: **2 driven wheels** (+ caster), or **4-wheel skid steer** with the left pair and the right pair driven together | Mecanum or omni wheels |
| **Wheel encoders** on the driven wheels | Car-like steering (a steered front axle) |
| **1 to 5 distance sensors**: VL53L0X, VL53L1X (short / long), Sharp GP2Y0A21 / GP2Y0A41, HC-SR04, or your own values | Robots without wheel encoders |

## The workflow

1. **Robot builder** — draw your robot from above: body, wheels, caster, where each sensor sits and where it looks.
   Choose sensors from the library (datasheet values filled in; change any of them). Export it as one JSON file
   (or as `robot_hardware.txt`) to keep it or edit it later.
2. **Measure** — connect the robot with a USB cable (Chrome or Edge) and let it measure itself step by step:
   encoder ticks per turn, top speed, motor feed-forward, wheel response time, real track width (spin test),
   sensor noise and bias, loop period and reading age. Every step explains what it measures, why it matters and
   what to do, and checks if the answer is plausible. No compatible firmware? Flash the small Arduino sketch
   offered on the page, or type the values yourself.
3. **Corridor** — an endless corridor with the errors of a real one (uneven walls, tilted boards, bumps, gaps,
   width steps — each on/off with its own size), or a track you draw (corners, S-bends) or import. Set the start
   pose (rotated up to ±80°, off-centre) and the noise.
4. **Train PID** — the computer searches Kp, Ki, Kd (and the speed) for *your* robot: a grid search, then a
   genetic algorithm, about 1–5 minutes. You get the crash rate and centring error at 0×, 1× and 2× noise, and
   ready-to-paste C code.
5. **Playground** — move the gain sliders while the robot drives. Presets show the classic mistakes (P too low /
   too high, no D, integral windup, too fast for the loop, wheel saturation). **Sweep** runs several values side
   by side as ghost robots from exactly the same start. The **Troubleshooting** page goes from "on the real robot it
   does X" to "try Y", each with a preset that reproduces it.
6. **PID tutorial** — PID step by step with small interactive demos.
7. **Maze sandbox** — random or hand-drawn mazes (thick or thin walls), built wall by wall with small errors that
   add up like in a real maze. Run wall following, flood fill or DFS mapping — or your own JavaScript — and see the
   robot's own map next to the real maze, with exit found, time, distance, crashes and map error.

**Loop period and delay:** the loop period is how often the robot decides (set by how fast the sensors deliver);
the reading age (delay) is how old the distance is when the robot uses it. You don't choose these — you measure
them. What you choose is the **speed**.

## Running it locally

Any static web server works, e.g. `python3 -m http.server -d web 8000`, then open <http://localhost:8000>.
(Opening the HTML file directly from disk does not work: browsers block WebAssembly and workers on `file://`.)

## Languages

English now. Hebrew and Arabic (right-to-left) are prepared: all text is in `web/i18n/*.json`; see
[DEVELOPERS.md](DEVELOPERS.md#translating).

## Where the physics comes from

The simulator is the C++ simulator of the **Simulation-To-Real-Life** project
(`Basel-Shokha/Simulation-To-Real-Life`, commit `efa1645a5d0f7aeebe9655f0cba7c57cd81a99a2`, copied to `vendor/`),
compiled to WebAssembly. Automatic tests check that the browser gives exactly the same numbers as the C++ program.
See [DEVELOPERS.md](DEVELOPERS.md).
