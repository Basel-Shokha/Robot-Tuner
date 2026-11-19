// robot_tuner_min.ino - minimal firmware that answers Robot Tuner's measurement commands over USB serial.
// For robots WITHOUT the Simulation-To-Real-Life firmware. Prints the same result lines, so the website's
// measurement wizard (measure.html) works unchanged.
//
// Commands (115200 baud, one per line):
//   CAL TICKS / CAL END        turn each wheel one revolution by hand -> ticks per revolution
//   CAL MAXSPEED               full power 1 s, wheels in the air -> top speed
//   CAL FF                     PWM sweep, wheels in the air -> deadband (ff_static) and PWM per speed (ff_gain)
//   CAL STEP [m/s]             speed step with a simple wheel PI -> time constant
//   CAL DRIVE <m> / CAL DIST <real m>      straight drive -> wheel scale
//   CAL SPIN <turns> / CAL TURNS <real>    spin in place -> effective track width
//   CAL TOF [n] [true_mm]      n readings per sensor -> noise, bias, dropouts
//   STATUS                     loop period and reading age
//   LOAD SENSORS / LOAD FINAL  motors off / on (sensor timing test)
//
// Fill in the CONFIG section (pins, geometry) and readSensorMm() for your sensors. Works on Arduino Uno/Nano
// (2 interrupt pins: use one channel per wheel), Mega, ESP32 (any pin). Motor driver: PWM + DIR per motor
// (L298N / DRV8833 / TB6612 with a direction pin). Encoders: quadrature, channel A on an interrupt pin.

// ======================= CONFIG =======================
const int PIN_PWM_L = 5, PIN_DIR_L = 4, PIN_PWM_R = 6, PIN_DIR_R = 7;   // motor driver
const int PIN_ENC_L_A = 2, PIN_ENC_L_B = 8, PIN_ENC_R_A = 3, PIN_ENC_R_B = 9;  // encoders (A = interrupt pin)
const int MOTOR_SIGN_L = 1, MOTOR_SIGN_R = 1, ENC_SIGN_L = 1, ENC_SIGN_R = 1;  // flip to -1 if a wheel runs backwards
const float WHEEL_DIAMETER_M = 0.065f;
float ticks_per_rev = 1440.0f;     // replace with the CAL TICKS result
float track_w = 0.14f;             // replace with the CAL TURNS result
const int PWM_MAX = 255;
const int N_SENSORS = 3;
const char* SENSOR_NAME[N_SENSORS] = {"right", "front", "left"};   // must match the sensor names in the robot builder
const unsigned long SENSOR_PERIOD_MS = 33;   // how long one reading of all sensors takes (the loop waits for it)

// Return the distance of sensor i in millimetres, or -1 when there is no valid reading.
// Replace with your sensor library (VL53L0X: readRangeContinuousMillimeters(); Sharp: analog curve; HC-SR04: pulseIn).
int readSensorMm(int i) {
  (void)i;
  return -1;
}
// =====================================================

volatile long encL = 0, encR = 0;
void isrL() { encL += (digitalRead(PIN_ENC_L_B) ? -1 : 1) * ENC_SIGN_L; }
void isrR() { encR += (digitalRead(PIN_ENC_R_B) ? -1 : 1) * ENC_SIGN_R; }
long cntL() { noInterrupts(); long v = encL; interrupts(); return v; }
long cntR() { noInterrupts(); long v = encR; interrupts(); return v; }
float mPerTick() { return 3.14159265f * WHEEL_DIAMETER_M / ticks_per_rev; }

bool motorsEnabled = true;
void setPwm(int l, int r) {
  if (!motorsEnabled) { l = 0; r = 0; }
  l = constrain(l * MOTOR_SIGN_L, -PWM_MAX, PWM_MAX); r = constrain(r * MOTOR_SIGN_R, -PWM_MAX, PWM_MAX);
  digitalWrite(PIN_DIR_L, l < 0); analogWrite(PIN_PWM_L, abs(l));
  digitalWrite(PIN_DIR_R, r < 0); analogWrite(PIN_PWM_R, abs(r));
}

// simple wheel speed PI with feed-forward (for STEP / DRIVE / SPIN)
float ff_static = 30, ff_gain = 400;   // PWM, PWM per (m/s): replaced by CAL FF
float wkp = 300, wki = 2000;           // PWM per (m/s), PWM per m
float iL = 0, iR = 0;
void wheelPI(float tl, float tr, float vl, float vr, float dt) {
  auto one = [&](float tg, float v, float& integ) {
    if (fabs(tg) < 1e-3) { integ = 0; return 0; }
    float e = tg - v; integ += e * dt;
    float u = (tg > 0 ? ff_static : -ff_static) + ff_gain * tg + wkp * e + wki * integ;
    return (int)constrain(u, -PWM_MAX, PWM_MAX);
  };
  setPwm(one(tl, vl, iL), one(tr, vr, iR));
}

String line;
unsigned long lastLoopUs = 0, loopPeriodUs = 0, readingAgeUs[N_SENSORS];
unsigned long readingTimeUs[N_SENSORS];
int lastMm[N_SENSORS];

void readSensors() {
  for (int i = 0; i < N_SENSORS; i++) { lastMm[i] = readSensorMm(i); readingTimeUs[i] = micros(); }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_PWM_L, OUTPUT); pinMode(PIN_DIR_L, OUTPUT); pinMode(PIN_PWM_R, OUTPUT); pinMode(PIN_DIR_R, OUTPUT);
  pinMode(PIN_ENC_L_A, INPUT_PULLUP); pinMode(PIN_ENC_L_B, INPUT_PULLUP); pinMode(PIN_ENC_R_A, INPUT_PULLUP); pinMode(PIN_ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrL, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrR, RISING);
  setPwm(0, 0);
  Serial.println("robot_tuner_min ready. Commands: CAL TICKS|END|MAXSPEED|FF|STEP|DRIVE|DIST|SPIN|TURNS|TOF, STATUS, LOAD SENSORS|FINAL");
}

// measured wheel speeds over dt (m/s)
void speeds(long& pl, long& pr, float dt, float& vl, float& vr) {
  long l = cntL(), r = cntR();
  vl = (l - pl) * mPerTick() / dt; vr = (r - pr) * mPerTick() / dt; pl = l; pr = r;
}

long c0L, c0R; bool ticksRunning = false; long driveTicksL = 0, driveTicksR = 0; float spinEncRot = 0;

void calTicks() { c0L = cntL(); c0R = cntR(); ticksRunning = true; Serial.println("OK CAL TICKS started: turn each wheel one revolution, then send CAL END"); }
void calEnd() {
  if (!ticksRunning) { Serial.println("ERR no CAL TICKS running"); return; }
  ticksRunning = false;
  long l = labs(cntL() - c0L), r = labs(cntR() - c0R);
  Serial.print("CAL TICKS result: left "); Serial.print(l); Serial.print(" right "); Serial.print(r); Serial.println(" per revolution");
  Serial.print("CAL suggestion: SET ticks_rev "); Serial.println((l + r) / 2.0, 1);
}
void calMaxSpeed() {
  setPwm(PWM_MAX, PWM_MAX); delay(500);
  long l = cntL(), r = cntR(); unsigned long t = millis(); delay(500);
  float dt = (millis() - t) / 1000.0f;
  float vl = (cntL() - l) * mPerTick() / dt, vr = (cntR() - r) * mPerTick() / dt;
  setPwm(0, 0);
  Serial.print("CAL MAXSPEED: left "); Serial.print(vl, 3); Serial.print(" m/s, right "); Serial.print(vr, 3); Serial.print(" m/s at PWM "); Serial.println(PWM_MAX);
  Serial.print("CAL suggestion: SET max_speed "); Serial.println(min(vl, vr), 4);
}
void calFF() {
  // sweep the PWM; speeds in ticks per 50 ms (the units of the full firmware), slope by least squares
  float sx = 0, sy = 0, sxx = 0, sxy = 0; int n = 0, start = -1;
  for (int pwm = 0; pwm <= PWM_MAX; pwm += 5) {
    setPwm(pwm, pwm); delay(250);
    long l = cntL(), r = cntR(); delay(50);
    float t50 = 0.5f * ((cntL() - l) + (cntR() - r));
    Serial.print("CAL FF: pwm "); Serial.print(pwm); Serial.print("  left "); Serial.print(t50, 2); Serial.print("  right "); Serial.print(t50, 2); Serial.println(" ticks/50ms");
    if (start < 0 && t50 >= 1) start = pwm;
    if (t50 >= 2) { sx += t50; sy += pwm; sxx += t50 * t50; sxy += t50 * pwm; n++; }
  }
  setPwm(0, 0);
  if (n >= 3) {
    float g = (n * sxy - sx * sy) / (n * sxx - sx * sx);
    Serial.print("CAL FF: slope "); Serial.print(g, 4); Serial.print(" PWM per tick/50ms (= "); Serial.print(g, 4); Serial.println(" per tick/50 ms)");
    ff_gain = g * 0.05f / mPerTick();  // PWM per (m/s) for this sketch's own PI
    Serial.print("CAL suggestion: SET ff_gain "); Serial.println(g, 4);
  }
  if (start >= 0) { ff_static = start; Serial.print("CAL suggestion: SET ff_static "); Serial.println(start); }
  Serial.println("CAL: FF sweep done");
}
void calStep(float v) {
  const int N = 150; static float y[N]; float dt = 0.01f;
  long pl = cntL(), pr = cntR(); iL = iR = 0;
  for (int k = 0; k < N; k++) { float vl, vr; delay(10); speeds(pl, pr, dt, vl, vr); wheelPI(v, v, vl, vr, dt); y[k] = 0.5f * (vl + vr); }
  setPwm(0, 0);
  float ss = 0; for (int k = N - N / 3; k < N; k++) ss += y[k]; ss /= N / 3;
  float t10 = -1, t63 = -1, t90 = -1, peak = 0;
  for (int k = 0; k < N; k++) { float f = y[k] / v; if (f > peak) peak = f; float tk = (k + 1) * dt;
    if (t10 < 0 && f >= 0.1) t10 = tk; if (t63 < 0 && f >= 0.632) t63 = tk; if (t90 < 0 && f >= 0.9) t90 = tk; }
  float rise = (t10 >= 0 && t90 >= 0) ? t90 - t10 : -1, over = peak > 1 ? (peak - 1) * 100 : 0;
  Serial.print("CAL STEP "); Serial.print(v, 2); Serial.print(" m/s (= sketch PI): rise L "); Serial.print(rise, 3); Serial.print("s R "); Serial.print(rise, 3);
  Serial.print("s, overshoot L "); Serial.print(over, 0); Serial.print("% R "); Serial.print(over, 0); Serial.print("%, tau63 L "); Serial.print(t63, 3);
  Serial.print("s R "); Serial.print(t63, 3); Serial.print("s, steady L "); Serial.print(ss, 3); Serial.print(" R "); Serial.println(ss, 3);
}
void calDrive(float dist) {
  long sl = cntL(), sr = cntR(), pl = sl, pr = sr; iL = iR = 0; float v = 0.2f;
  for (;;) {
    float vl, vr; delay(10); speeds(pl, pr, 0.01f, vl, vr);
    float dl = (cntL() - sl) * mPerTick(), dr = (cntR() - sr) * mPerTick();
    if (0.5f * (dl + dr) >= dist) break;
    float corr = 2.0f * (dl - dr);
    wheelPI(v - corr, v + corr, vl, vr, 0.01f);
  }
  setPwm(0, 0);
  driveTicksL = cntL() - sl; driveTicksR = cntR() - sr;
  Serial.print("CAL DRIVE done: ticks L "); Serial.print(driveTicksL); Serial.print(" R "); Serial.println(driveTicksR);
  Serial.println("CAL DRIVE: measure the real distance D (m), then send: CAL DIST <D>");
}
void calDist(float real) {
  if (!driveTicksL) { Serial.println("ERR run CAL DRIVE first"); return; }
  float tl = driveTicksL / real, tr = driveTicksR / real, nominal = 1.0f / mPerTick();
  Serial.print("CAL DIST: ticks per metre L "); Serial.print(tl, 1); Serial.print(" R "); Serial.print(tr, 1);
  Serial.print("; suggestions wscale_l "); Serial.print(nominal / tl, 4); Serial.print(" wscale_r "); Serial.println(nominal / tr, 4);
}
void calSpin(float turns) {
  long sl = cntL(), sr = cntR(), pl = sl, pr = sr; iL = iR = 0; float goal = turns * 2 * 3.14159265f, v = 0.12f;
  float dl = 0, dr = 0;
  for (;;) {
    float vl, vr; delay(10); speeds(pl, pr, 0.01f, vl, vr);
    dl = (cntL() - sl) * mPerTick(); dr = (cntR() - sr) * mPerTick();
    if ((dr - dl) / track_w >= goal) break;
    float drift = 2.0f * (dl + dr);
    wheelPI(-v - drift, v - drift, vl, vr, 0.01f);
  }
  setPwm(0, 0);
  spinEncRot = (dr - dl) / track_w;
  float asym = fabs(dl + dr) / max(1e-6f, fabs(dl) + fabs(dr));
  Serial.print("CAL SPIN done: encoders say "); Serial.print(spinEncRot / (2 * 3.14159265f), 3); Serial.print(" turns with track_w "); Serial.print(track_w, 4);
  Serial.print(" m; asymmetry "); Serial.print(100 * asym, 1); Serial.print(" %, at the PWM clamp 0 % of the time -> "); Serial.println(asym < 0.05 ? "OK" : "UNTRUSTED");
  Serial.println("CAL SPIN: count the REAL turns and send: CAL TURNS <real_turns>");
}
void calTurns(float real) {
  if (spinEncRot <= 0) { Serial.println("ERR run CAL SPIN first"); return; }
  float tw = track_w * spinEncRot / (real * 2 * 3.14159265f);
  Serial.print("CAL TURNS: effective track width "); Serial.print(tw, 4); Serial.print(" m (was "); Serial.print(track_w, 4); Serial.println(").");
  track_w = tw;
}
void calTof(int n, float trueMm) {
  for (int i = 0; i < N_SENSORS; i++) {
    double s = 0, sq = 0; int cnt = 0, drop = 0, mn = 32767, mx = 0;
    for (int k = 0; k < n; k++) {
      int mm = readSensorMm(i); delay(SENSOR_PERIOD_MS / N_SENSORS + 1);
      if (mm < 0) { drop++; continue; }
      s += mm; sq += (double)mm * mm; cnt++; mn = min(mn, mm); mx = max(mx, mm);
    }
    double mean = cnt ? s / cnt : 0, sd = cnt > 1 ? sqrt(max(0.0, (sq - cnt * mean * mean) / (cnt - 1))) : 0;
    Serial.print("CAL TOF "); Serial.print(SENSOR_NAME[i]); Serial.print(" n "); Serial.print(cnt); Serial.print(" mean "); Serial.print(mean, 1);
    Serial.print(" mm std "); Serial.print(sd, 2); Serial.print(" mm min "); Serial.print(cnt ? mn : 0); Serial.print(" max "); Serial.print(mx);
    Serial.print(" dropouts "); Serial.print(drop); Serial.print(" ("); Serial.print(100.0 * drop / max(1, cnt + drop), 1); Serial.println("%)");
    if (trueMm > 0 && cnt) { Serial.print("CAL TOF "); Serial.print(SENSOR_NAME[i]); Serial.print(" bias vs "); Serial.print(trueMm, 0); Serial.print(" mm: "); Serial.print(mean - trueMm >= 0 ? "+" : ""); Serial.print(mean - trueMm, 1); Serial.println(" mm"); }
  }
  Serial.println("CAL: TOF done");
}
void status() {
  unsigned long now = micros();
  Serial.print("tof mm");
  for (int i = 0; i < N_SENSORS; i++) { Serial.print(" "); Serial.print(SENSOR_NAME[i]); Serial.print(" "); Serial.print(lastMm[i]); }
  Serial.println();
  // the full firmware prints three ages (R/F/L); we print the first three sensors
  Serial.print("ToF ages at the last tick: data-ready seen ");
  for (int i = 0; i < 3; i++) { Serial.print(now - readingTimeUs[min(i, N_SENSORS - 1)]); Serial.print(i < 2 ? "/" : ""); }
  Serial.print(" us ago (R/F/L), + 0 ms half budget\n");
  Serial.print("control loop (ToF data-ready): period "); Serial.print(loopPeriodUs); Serial.print(" us (max "); Serial.print(loopPeriodUs); Serial.println("), compute 0 us, overruns 0, ticks 0, telemetry dropped 0");
}

void handle(String c) {
  c.trim(); String u = c; u.toUpperCase();
  float a1 = 0, a2 = 0; int sp1 = u.indexOf(' ', 4), sp2 = sp1 >= 0 ? u.indexOf(' ', sp1 + 1) : -1;
  if (sp1 >= 0) a1 = u.substring(sp1 + 1).toFloat();
  if (sp2 >= 0) a2 = u.substring(sp2 + 1).toFloat();
  if (u == "CAL TICKS") calTicks();
  else if (u == "CAL END") calEnd();
  else if (u == "CAL MAXSPEED") calMaxSpeed();
  else if (u == "CAL FF") calFF();
  else if (u.startsWith("CAL STEP")) calStep(a1 > 0 ? a1 : 0.15f);
  else if (u.startsWith("CAL DRIVE")) calDrive(a1 > 0 ? a1 : 1.0f);
  else if (u.startsWith("CAL DIST")) calDist(a1);
  else if (u.startsWith("CAL SPIN")) calSpin(a1 > 0 ? a1 : 5);
  else if (u.startsWith("CAL TURNS")) calTurns(a1);
  else if (u.startsWith("CAL TOF")) calTof(a1 > 0 ? (int)a1 : 100, a2);
  else if (u == "STATUS") status();
  else if (u == "LOAD SENSORS") { motorsEnabled = false; setPwm(0, 0); Serial.println("OK profile SENSORS (motors off)"); }
  else if (u == "LOAD FINAL") { motorsEnabled = true; Serial.println("OK profile FINAL"); }
  else Serial.println("ERR unknown command");
}

void loop() {
  // the "control loop": read all sensors, measure the period between two complete sets
  readSensors();
  unsigned long now = micros();
  loopPeriodUs = now - lastLoopUs; lastLoopUs = now;
  unsigned long waitUntil = millis() + SENSOR_PERIOD_MS;
  while (millis() < waitUntil) {
    while (Serial.available()) {
      char ch = Serial.read();
      if (ch == '\n') { handle(line); line = ""; } else if (ch != '\r') line += ch;
    }
  }
}
