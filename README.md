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
| **Wheel encoders** on the driven wheels (quadrature, single-channel or AS5600); without encoders only a limited time-based mode | Car-like steering (a steered front axle) |
| **1 to 5 distance sensors**: VL53L0X, VL53L1X (short / long), Sharp GP2Y0A21 / GP2Y0A41, HC-SR04, or your own values | |

See [Supported hardware](#supported-hardware) for the boards, encoders and motor drivers the sketch generator handles.

## The workflow

1. **Robot builder** — draw your robot from above: body, wheels, caster, where each sensor sits and where it looks.
   Choose sensors from the library (datasheet values filled in; change any of them). Export it as one JSON file
   (or as `robot_hardware.txt`) to keep it or edit it later. The **Firmware** tab asks how the robot is wired
   (board, encoders, motor driver, sensor pins), checks the pins, and downloads a ready-to-flash sketch.
2. **Measure** — connect the robot with a USB cable (Chrome or Edge) and let it measure itself step by step:
   encoder ticks per turn, top speed, slowest steady speed (v_min), motor feed-forward, wheel response time, real track width (spin test),
   sensor noise and bias, loop period and reading age. Every step explains what it measures, why it matters and
   what to do, and checks if the answer is plausible. No compatible firmware? The robot builder's **Firmware** tab
   generates a ready-to-flash Arduino sketch for your wiring, or type the values yourself.
3. **Corridor** — an endless corridor with the errors of a real one (uneven walls, tilted boards, bumps, gaps,
   width steps — each on/off with its own size), or a track you draw (corners, S-bends) or import. Set the start
   pose (rotated up to ±80°, off-centre) and the noise.
4. **Train PID** — choose the cruise speed (`speed_frac`), then the computer searches Kp, Ki, Kd for *your* robot at that speed: a grid search, then a
   genetic algorithm, about 1–5 minutes. You get the crash rate and centring error at 0×, 1× and 2× noise, and
   ready-to-paste C code.
5. **Playground** — move the gain and `speed_frac` sliders while the robot drives. A wheel whose command is cut at v_max turns red, with the asked vs real turn next to it. Presets show the classic mistakes (P too low /
   too high, no D, integral windup, too fast for the loop, wheel saturation). **Sweep** runs several values side
   by side as ghost robots from exactly the same start. The **Troubleshooting** page goes from "on the real robot it
   does X" to "try Y", each with a preset that reproduces it.
6. **PID tutorial** — PID step by step with small interactive demos.
7. **Maze sandbox** — random or hand-drawn mazes (thick or thin walls), built wall by wall with small errors that
   add up like in a real maze. Run wall following, flood fill or DFS mapping — or your own JavaScript — and see the
   robot's own map next to the real maze, with exit found, time, distance, crashes and map error.

**The PID** (the same arithmetic in the simulator, the exported `pid_gains.h` and the generated sketch):

```
e        = clamp(d_left - d_right, ±error_clip)            left / right = the ±45° sensors
integral = clamp(integral + e·dt, ±integral_limit)          anti-windup
d_filt   = α·d_raw + (1 − α)·d_filt                         α given per 50 ms, adjusted to the tick time
turn     = clamp(Kp·e + Ki·integral + Kd·d_filt, ±v_max)
base     = v_min + speed_frac·(v_max − v_min)               constant cruise speed, speed_frac default 0.75
v_left   = clamp(base − turn, ±v_max),  v_right = clamp(base + turn, ±v_max)
```

This is upstream `PID/PidController.hpp` in its constant-speed mode (`cs_mode 1` / `const_speed 1`, defaults
`cs_speed_frac 0.75`, `cs_v_min_mps 0.013`). There is no slowdown rule and no speed law. v_max is the measured top
speed, v_min the measured slowest steady speed (0.013 m/s, the upstream default, until measured). The higher `speed_frac`, the less room is left above `base` to turn: at 1.0 every turn
clips the outer wheel (Playground preset "Same gains: 1.0 vs 0.75").

**Loop period and delay:** the loop period is how often the robot decides (set by how fast the sensors deliver);
the reading age (delay) is how old the distance is when the robot uses it. You don't choose these — you measure
them. What you choose is the **speed**.

## Supported hardware

The robot builder's **Firmware** tab generates `robot_tuner_min.ino` for the wiring you enter: pins, libraries and
code. It answers the same serial commands as before (`CAL TICKS`, `CAL FF`, `CAL TOF`, `STATUS`, ...), so the Measure
page works the same for every robot. It checks the pins (interrupt / PWM / analog ability, pins used twice, USB
serial, I2C, ESP32 strapping and input-only pins) and can fill in free pins for you. Every combination is compiled
with arduino-cli in CI (`make firmware`, `tools/compile_firmware.sh`).

| Part | Option | How the sketch does it | Library (Library Manager) | Notes |
|---|---|---|---|---|
| **Board** | Arduino Uno / Nano | `arduino:avr:uno` | — | attachInterrupt only on D2 / D3; PWM on D3, D5, D6, D9–D11; I2C on A4 / A5. 3× VL53L1X + all features fit (≤ 95 % flash) |
| | Arduino Mega 2560 | `arduino:avr:mega` | — | attachInterrupt on D2, D3, D18–D21; I2C on D20 / D21 |
| | ESP32 (WROOM-32 DevKit) | `esp32:esp32:esp32`, core 3.x or 4.x | — | GPIO 34–39 input only (no pull-ups), 0 / 2 / 5 / 12 / 15 strapping (warned), 6–11 flash (not offered) |
| **Encoders** | Quadrature A + B (Hall / optical) | ESP32: hardware pulse counter (PCNT, ESP-IDF `pulse_cnt` driver, x4). Uno / Mega: x1 (A on an interrupt pin, B read) or x4 (all edges) | `PinChangeInterrupt` (x4 on non-interrupt pins) | ticks per wheel turn = PPR × edges × gear ratio |
| | Single channel (LM393 slotted disc, TT motors) | pulses counted on an interrupt, sign from the commanded motor direction, debounced | `PinChangeInterrupt` (if not on D2 / D3) | every reversal miscounts the pulses while the wheel still turns the old way (a few ticks each) |
| | AS5600 magnetic (I2C) | angle read every 0.5 ms, turns counted from the wrap | — | two AS5600 share address 0x36: TCA9548A multiplexer, or a second I2C bus on the ESP32 |
| | None | time-based mode: encoder commands answer `ERR`, `CAL TIMED` drives at full power for a set time | — | Measure page disables the encoder steps; turns drift |
| | < 100 ticks per wheel turn | wheel speed from the time between pulses, not pulses per interval | — | automatic |
| **Motor driver** | Two inputs IN1 / IN2: DRV8833, MX1508, L9110S, L298N with ENA jumper | both inputs PWM; brake = slow decay, coast = fast decay | — | brake / coast choice |
| | PWM + DIR: DRV8838, DRV8835 (phase / enable), Cytron MD10C | PWM pin + direction pin | — | the driver decides brake / coast |
| | EN + IN1 + IN2: L298N with ENA, TB6612FNG | PWM on EN, direction on IN1 / IN2, optional STBY held HIGH | — | brake / coast choice |
| **Distance sensors** | VL53L0X, VL53L1X (short / long) | continuous mode, read when data is ready; several on one bus: XSHUT pins hold them off, then each gets its own address | `VL53L0X`, `VL53L1X` (Pololu) | number, position and angle from the Sensors tab (also used by the simulator) |
| | HC-SR04 | one sensor pings at a time (no crosstalk), echo timeout from the max range | — | slow: a set of N sensors takes N × (echo + gap); 5 V ECHO needs a divider on the ESP32 |
| | Sharp GP2Y0A21 / GP2Y0A41 | analog, power-law curve fit of the datasheet (`cm = A·V^-B`) | — | check with a ruler (`CAL TOF` bias) |

Not supported by the generator: 4-wheel independent drive, mecanum, more than two drive channels, I2C multiplexers
for the ToF sensors themselves, ESP32-S2 / S3 / C3 boards (other pin maps; the code itself would mostly work), and
magnetic or optical encoders that need their own library (SPI AS5047, I2C AS5048).

## Running it on your own computer

Download the repository (Code → Download ZIP), unzip it, then:

- **Windows:** double-click `tune.bat`
- **macOS:** double-click `tune.command` (first time: right-click → Open)
- **Linux / anyone with make:** `make tune` (or `./tune.sh`)

It starts a small local web server and opens Robot Tuner in Chrome. Close the window (or press Ctrl+C) to stop.
You need Python 3 (preinstalled on macOS and Linux; on Windows get it from python.org and tick "Add python.exe to
PATH"). Use Chrome or Edge for the USB measurement page. Opening `web/index.html` directly does not work: browsers
block WebAssembly and workers on `file://`.

## Languages

English now. Hebrew and Arabic (right-to-left) are prepared: all text is in `web/i18n/*.json`; see
[DEVELOPERS.md](DEVELOPERS.md#translating).

## Where the physics comes from

The simulator is the C++ simulator of the **Simulation-To-Real-Life** project
(`Basel-Shokha/Simulation-To-Real-Life`, commit `a929bcb86d9668703cdff2d39b76cbd67436d9a8`, copied to `vendor/`),
compiled to WebAssembly. Automatic tests check that the browser gives exactly the same numbers as the C++ program.
See [DEVELOPERS.md](DEVELOPERS.md).
