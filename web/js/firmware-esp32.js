// firmware-esp32.js - the ESP32 "install" firmware: ONE compiled firmware for every ESP32 robot. Pins, driver, encoders,
// sensors, ticks, gains and Bluetooth are SETTINGS kept in the ESP32's flash (NVS), set over serial with the robot
// file's keys:  SET <key> <value>  ...  SAVE  (stores and restarts).  GET prints them.  The measuring commands are the
// same code as the generated sketch (firmware-gen.js CORE), so the Measure page works unchanged.
//
// CI compiles it (tools/gen_firmware_combos.mjs writes it next to the generated sketches; pages.yml publishes the .bin)
// and the site flashes it from the browser (install.js, esptool-js).
import { CORE, BAUD, SHARP_FIT } from './firmware-gen.js';

/** the robot-file keys the firmware understands (everything else in a robot file is ignored by SET) */
export const FW_KEYS = [
  'wheel_diameter_mm', 'wheel_track_mm', 'track_w_effective_mm', 'enc_ticks_per_rev', 'max_speed_mps', 'v_min_mps',
  'pid_kp', 'pid_ki', 'pid_kd', 'pid_speed_frac', 'pid_error_clip_mm', 'pid_integral_limit', 'pid_d_filter_alpha', 'pid_offset_mm',
  'motor_driver', 'motor_stop', 'pin_motor_left_a', 'pin_motor_left_b', 'pin_motor_left_en', 'pin_motor_right_a', 'pin_motor_right_b',
  'pin_motor_right_en', 'pin_motor_stby', 'motor_left_sign', 'motor_right_sign',
  'encoder_type', 'pin_enc_left_a', 'pin_enc_left_b', 'pin_enc_right_a', 'pin_enc_right_b', 'enc_left_sign', 'enc_right_sign',
  'pin_i2c_sda', 'pin_i2c_scl', 'bluetooth', 'bluetooth_name', 'sensor_count',
  ...[1, 2, 3, 4, 5].flatMap((n) => ['type', 'name', 'angle_deg', 'max_mm', 'min_mm', 'period_ms', 'xshut', 'addr', 'trig', 'echo', 'analog'].map((f) => `sensor${n}_${f}`)),
];

const f3 = (x) => (+x).toFixed(3) + 'f';
const PRELUDE = () => String.raw`// robot_tuner_esp32.ino - Robot Tuner install firmware for every ESP32 robot (generated; do not edit).
// Settings over serial / Bluetooth: SET <key> <value> (robot-file keys), GET, SAVE (stores in flash and restarts).
// Until the settings are saved, the motors stay off and the commands that need them answer "ERR not set up".
#include <Wire.h>
#include <Preferences.h>
#include <VL53L0X.h>
#include <VL53L1X.h>
#include "BluetoothSerial.h"

struct Meter { long pl, pr; unsigned long us; };
#define HAS_ENCODERS 1
#define LOW_RES 0
#define MAXS 5
const int PWM_MAX = 255;

// ---------------- settings (key value text, kept in NVS) ----------------
Preferences prefs;
String cfgText;                       // "key value\n" lines
String cfgGet(const char* key, const char* def = "") {
  String k = String(key) + " "; int p = 0, n = cfgText.length();
  while (p < n) {
    int e = cfgText.indexOf('\n', p); if (e < 0) e = n;
    if (cfgText.substring(p, e).startsWith(k)) { String v = cfgText.substring(p + k.length(), e); v.trim(); return v.length() ? v : String(def); }
    p = e + 1;
  }
  return String(def);
}
float cfgF(const char* key, float def) { String v = cfgGet(key); return v.length() && v != "N/A" ? v.toFloat() : def; }
int cfgPin(const char* key) { String v = cfgGet(key); if (!v.length() || v == "N/A") return -1; return v.startsWith("0x") || v.startsWith("0X") ? (int)strtol(v.c_str(), NULL, 16) : v.toInt(); }
void cfgSet(const String& key, const String& val) {
  String out, line; int p = 0;
  while (p < (int)cfgText.length()) { int e = cfgText.indexOf('\n', p); if (e < 0) e = cfgText.length(); line = cfgText.substring(p, e); p = e + 1;
    if (line.length() && !line.startsWith(key + " ")) out += line + "\n"; }
  cfgText = out + key + " " + val + "\n";
}
bool configured = false;

// ---------------- I/O: USB and Bluetooth ----------------
BluetoothSerial SerialBT; bool btOn = false;
class TeeIO : public Stream {
 public:
  size_t write(uint8_t c) override { USB_SERIAL_.write(c); if (btOn) SerialBT.write(c); return 1; }
  size_t write(const uint8_t* b, size_t n) override { USB_SERIAL_.write(b, n); if (btOn) SerialBT.write(b, n); return n; }
  int available() override { return USB_SERIAL_.available() + (btOn ? SerialBT.available() : 0); }
  int read() override { return USB_SERIAL_.available() ? USB_SERIAL_.read() : (btOn ? SerialBT.read() : -1); }
  int peek() override { return USB_SERIAL_.available() ? USB_SERIAL_.peek() : (btOn ? SerialBT.peek() : -1); }
  void flush() override { USB_SERIAL_.flush(); if (btOn) SerialBT.flush(); }
};
TeeIO RT_IO;

// ---------------- robot numbers (from the settings) ----------------
float WHEEL_DIAMETER_M = 0.045f, ticks_per_rev = 1000, track_w = 0.12f;
int N_SENSORS = 0;
const char* SENSOR_NAME[MAXS] = {"s1", "s2", "s3", "s4", "s5"};
String sensorNames[MAXS];
float PID_KP = 0, PID_KI = 0, PID_KD = 0, PID_SPEED_FRAC = 0.5f, PID_V_MAX = 0.2f, PID_V_MIN = 0, PID_ERROR_CLIP = 0.08f, PID_INTEGRAL_LIMIT = 0.5f, PID_D_ALPHA = 0.3f, PID_SETPOINT = 0;
int PID_LEFT = 0, PID_RIGHT = 0;
float SENSOR_MAX_M[MAXS];
#define CAN_RUN (configured && N_SENSORS >= 2 && PID_LEFT != PID_RIGHT)
float mPerTick() { return 3.14159265f * WHEEL_DIAMETER_M / ticks_per_rev; }

// ---------------- motors ----------------
bool motorsEnabled = true;
volatile int cmdDirL = 1, cmdDirR = 1;
int drvType = 0, pinA[2], pinB[2], pinEn[2], pinStby = -1, motorSign[2] = {1, 1}; bool stopBrake = true;   // drvType 0 in1_in2, 1 pwm_dir, 2 en_in1_in2
void motorOut(int s, int u) {
  if (pinA[s] < 0 || pinB[s] < 0) return;
  int d = constrain(abs(u), 0, PWM_MAX);
  if (drvType == 1) { digitalWrite(pinB[s], u < 0 ? HIGH : LOW); analogWrite(pinA[s], d); return; }
  if (drvType == 2) {
    if (pinEn[s] < 0) return;
    if (d == 0) { digitalWrite(pinA[s], stopBrake); digitalWrite(pinB[s], stopBrake); analogWrite(pinEn[s], stopBrake ? PWM_MAX : 0); return; }
    digitalWrite(pinA[s], u > 0 ? HIGH : LOW); digitalWrite(pinB[s], u > 0 ? LOW : HIGH); analogWrite(pinEn[s], d); return;
  }
  if (d == 0) { analogWrite(pinA[s], stopBrake ? PWM_MAX : 0); analogWrite(pinB[s], stopBrake ? PWM_MAX : 0); return; }
  if (stopBrake) { if (u > 0) { analogWrite(pinA[s], PWM_MAX); analogWrite(pinB[s], PWM_MAX - d); } else { analogWrite(pinA[s], PWM_MAX - d); analogWrite(pinB[s], PWM_MAX); } }
  else { if (u > 0) { analogWrite(pinA[s], d); analogWrite(pinB[s], 0); } else { analogWrite(pinA[s], 0); analogWrite(pinB[s], d); } }
}
void setPwm(int l, int r) {
  if (!motorsEnabled || !configured) { l = 0; r = 0; }
  l = constrain(l, -PWM_MAX, PWM_MAX); r = constrain(r, -PWM_MAX, PWM_MAX);
  if (l) cmdDirL = l > 0 ? 1 : -1;
  if (r) cmdDirR = r > 0 ? 1 : -1;
  motorOut(0, l * motorSign[0]); motorOut(1, r * motorSign[1]);
}

// ---------------- encoders (interrupts; quadrature x4 or single channel) ----------------
volatile long encCount[2] = {0, 0}; volatile uint8_t qState[2] = {0, 0};
int encType = 0, encA[2] = {-1, -1}, encB[2] = {-1, -1}, encSign[2] = {1, 1};   // encType 0 quadrature, 1 single, 2 none
const int8_t QDEC[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
void IRAM_ATTR encIsr(void* arg) {
  int s = (int)(intptr_t)arg;
  if (encType == 1) { encCount[s] += s ? cmdDirR : cmdDirL; return; }
  uint8_t st = (digitalRead(encA[s]) << 1) | digitalRead(encB[s]);
  encCount[s] += QDEC[(qState[s] << 2) | st]; qState[s] = st;
}
long cntL() { noInterrupts(); long v = encCount[0]; interrupts(); return encType == 1 ? v : v * encSign[0]; }
long cntR() { noInterrupts(); long v = encCount[1]; interrupts(); return encType == 1 ? v : v * encSign[1]; }
void encPoll() {}

// ---------------- distance sensors ----------------
int lastMm[MAXS]; unsigned long readingTimeUs[MAXS], sensorSeq[MAXS]; bool freshInSet[MAXS];
unsigned long lastSetUs = 0, loopPeriodUs = 0; bool setReady = false;
void newReading(int i, int mm) {
  lastMm[i] = mm; readingTimeUs[i] = micros(); sensorSeq[i]++; freshInSet[i] = true;
  for (int k = 0; k < N_SENSORS; k++) if (!freshInSet[k]) return;
  unsigned long now = micros(); loopPeriodUs = now - lastSetUs; lastSetUs = now; setReady = true;
  for (int k = 0; k < N_SENSORS; k++) freshInSet[k] = false;
}
int sKind[MAXS];   // 0 none, 1 VL53L0X, 2 VL53L1X short, 3 VL53L1X long, 4 HC-SR04, 5 GP2Y0A21, 6 GP2Y0A41
VL53L0X tofA[MAXS]; VL53L1X tofB[MAXS]; bool tofOk[MAXS];
int sXshut[MAXS], sAddr[MAXS], sTrig[MAXS], sEcho[MAXS], sAnalog[MAXS], sPeriod[MAXS], sMinMm[MAXS], sMaxMm[MAXS];
unsigned long sLastMs[MAXS], lastTofPollUs = 0; int sonarTurn = 0; unsigned long sonarLastMs = 0;
int kindOf(String t) {
  t.toUpperCase();
  if (t == "VL53L0X") return 1; if (t == "VL53L1X_SHORT") return 2; if (t == "VL53L1X_LONG") return 3;
  if (t == "HC_SR04") return 4; if (t == "GP2Y0A21") return 5; if (t == "GP2Y0A41") return 6; return 0;
}
void sensorsSetup() {
  for (int i = 0; i < MAXS; i++) { lastMm[i] = -1; readingTimeUs[i] = micros(); sensorSeq[i] = 0; freshInSet[i] = false; sLastMs[i] = 0; tofOk[i] = false; }
  for (int i = 0; i < N_SENSORS; i++) if (sKind[i] >= 1 && sKind[i] <= 3 && sXshut[i] >= 0) { pinMode(sXshut[i], OUTPUT); digitalWrite(sXshut[i], LOW); }
  delay(10);
  for (int i = 0; i < N_SENSORS; i++) {
    int k = sKind[i], addr = sAddr[i] > 0 ? sAddr[i] : 0x30 + i;
    if (k >= 1 && k <= 3) {
      if (sXshut[i] >= 0) { pinMode(sXshut[i], INPUT); delay(10); }
      if (k == 1) {
        tofA[i].setTimeout(500); tofOk[i] = tofA[i].init();
        if (tofOk[i]) { tofA[i].setAddress(addr); tofA[i].setMeasurementTimingBudget(max(20, sPeriod[i] - 2) * 1000); tofA[i].startContinuous(0); }
      } else {
        tofB[i].setTimeout(500); tofOk[i] = tofB[i].init();
        if (tofOk[i]) {
          tofB[i].setAddress(addr); tofB[i].setDistanceMode(k == 3 ? VL53L1X::Long : VL53L1X::Short);
          int budget = max(k == 3 ? 33 : 20, sPeriod[i] - 5);
          tofB[i].setMeasurementTimingBudget(budget * 1000); tofB[i].startContinuous(max(sPeriod[i], budget + 4));
        }
      }
      if (!tofOk[i]) { RT_IO.print(F("WARN sensor ")); RT_IO.print(SENSOR_NAME[i]); RT_IO.println(F(" not found: check wiring and XSHUT")); }
    }
    if (k == 4 && sTrig[i] >= 0 && sEcho[i] >= 0) { pinMode(sTrig[i], OUTPUT); digitalWrite(sTrig[i], LOW); pinMode(sEcho[i], INPUT); }
    if ((k == 5 || k == 6) && sAnalog[i] >= 0) pinMode(sAnalog[i], INPUT);
  }
}
void pollSensors() {
  if (micros() - lastTofPollUs >= 1000) {
    lastTofPollUs = micros();
    for (int i = 0; i < N_SENSORS; i++) {
      int k = sKind[i];
      if (k < 1 || k > 3) continue;
      if (!tofOk[i]) { if (millis() - sLastMs[i] >= (unsigned long)sPeriod[i]) { sLastMs[i] = millis(); newReading(i, -1); } continue; }
      if (k == 1) { if (tofA[i].readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) { uint16_t mm = tofA[i].readRangeContinuousMillimeters(); newReading(i, (tofA[i].timeoutOccurred() || mm >= 8000) ? -1 : (int)mm); } }
      else if (tofB[i].dataReady()) { tofB[i].read(false); newReading(i, tofB[i].ranging_data.range_status == VL53L1X::RangeValid ? (int)tofB[i].ranging_data.range_mm : -1); }
    }
  }
  for (int i = 0; i < N_SENSORS; i++) {
    int k = sKind[i];
    if ((k == 5 || k == 6) && sAnalog[i] >= 0 && millis() - sLastMs[i] >= (unsigned long)sPeriod[i]) {
      sLastMs[i] = millis();
      long s = 0; for (int q = 0; q < 4; q++) s += analogReadMilliVolts(sAnalog[i]);
      float v = s / 4000.0f, a = k == 5 ? ${f3(SHARP_FIT.GP2Y0A21.a)} : ${f3(SHARP_FIT.GP2Y0A41.a)}, b = k == 5 ? ${f3(SHARP_FIT.GP2Y0A21.b)} : ${f3(SHARP_FIT.GP2Y0A41.b)};
      float mm = v > 0.1f ? 10.0f * a * pow(v, -b) : -1;
      newReading(i, (mm < sMinMm[i] || mm > sMaxMm[i]) ? -1 : (int)mm);
    }
  }
  // HC-SR04: one at a time (round robin), quiet gap so echoes die out
  int nSonar = 0; for (int i = 0; i < N_SENSORS; i++) if (sKind[i] == 4) nSonar++;
  if (nSonar && millis() - sonarLastMs >= (unsigned long)max(10, 60 / nSonar)) {
    for (int tries = 0; tries < N_SENSORS; tries++) {
      int i = sonarTurn; sonarTurn = (sonarTurn + 1) % N_SENSORS;
      if (sKind[i] != 4 || sTrig[i] < 0 || sEcho[i] < 0) continue;
      digitalWrite(sTrig[i], LOW); delayMicroseconds(2); digitalWrite(sTrig[i], HIGH); delayMicroseconds(10); digitalWrite(sTrig[i], LOW);
      unsigned long us = pulseIn(sEcho[i], HIGH, (unsigned long)(sMaxMm[i] * 2 / 0.343f + 1000));
      newReading(i, us ? (int)(us * 0.1715f) : -1);
      break;
    }
    sonarLastMs = millis();
  }
}

// ---------------- apply the settings (at start-up) ----------------
void loadSettings() {
  prefs.begin("robottuner", false);
  cfgText = prefs.getString("cfg", "");
  WHEEL_DIAMETER_M = cfgF("wheel_diameter_mm", 45) / 1000.0f;
  ticks_per_rev = cfgF("enc_ticks_per_rev", 1000);
  track_w = cfgF("track_w_effective_mm", cfgF("wheel_track_mm", 120)) / 1000.0f;
  String d = cfgGet("motor_driver", "in1_in2"); drvType = d == "pwm_dir" ? 1 : d == "en_in1_in2" ? 2 : 0;
  stopBrake = cfgGet("motor_stop", "brake") != "coast";
  pinA[0] = cfgPin("pin_motor_left_a"); pinB[0] = cfgPin("pin_motor_left_b"); pinEn[0] = cfgPin("pin_motor_left_en");
  pinA[1] = cfgPin("pin_motor_right_a"); pinB[1] = cfgPin("pin_motor_right_b"); pinEn[1] = cfgPin("pin_motor_right_en");
  pinStby = cfgPin("pin_motor_stby");
  motorSign[0] = cfgF("motor_left_sign", 1) < 0 ? -1 : 1; motorSign[1] = cfgF("motor_right_sign", 1) < 0 ? -1 : 1;
  String e = cfgGet("encoder_type", "quadrature"); encType = e == "single" ? 1 : e == "quadrature" ? 0 : 2;
  encA[0] = cfgPin("pin_enc_left_a"); encB[0] = cfgPin("pin_enc_left_b"); encA[1] = cfgPin("pin_enc_right_a"); encB[1] = cfgPin("pin_enc_right_b");
  encSign[0] = cfgF("enc_left_sign", 1) < 0 ? -1 : 1; encSign[1] = cfgF("enc_right_sign", 1) < 0 ? -1 : 1;
  N_SENSORS = constrain((int)cfgF("sensor_count", 0), 0, MAXS);
  float maxA = -1e9, minA = 1e9;
  for (int i = 0; i < N_SENSORS; i++) {
    char k[24]; int n = i + 1;
    snprintf(k, sizeof k, "sensor%d_type", n); sKind[i] = kindOf(cfgGet(k));
    snprintf(k, sizeof k, "sensor%d_name", n); sensorNames[i] = cfgGet(k, ""); if (!sensorNames[i].length()) sensorNames[i] = String("s") + n; SENSOR_NAME[i] = sensorNames[i].c_str();
    snprintf(k, sizeof k, "sensor%d_angle_deg", n); float a = cfgF(k, 0);
    if (a > maxA) { maxA = a; PID_LEFT = i; } if (a < minA) { minA = a; PID_RIGHT = i; }
    snprintf(k, sizeof k, "sensor%d_max_mm", n); sMaxMm[i] = (int)cfgF(k, 1300); SENSOR_MAX_M[i] = sMaxMm[i] / 1000.0f;
    snprintf(k, sizeof k, "sensor%d_min_mm", n); sMinMm[i] = (int)cfgF(k, 30);
    snprintf(k, sizeof k, "sensor%d_period_ms", n); sPeriod[i] = max(17, (int)cfgF(k, 33));
    snprintf(k, sizeof k, "sensor%d_xshut", n); sXshut[i] = cfgPin(k);
    snprintf(k, sizeof k, "sensor%d_addr", n); sAddr[i] = cfgPin(k);
    snprintf(k, sizeof k, "sensor%d_trig", n); sTrig[i] = cfgPin(k);
    snprintf(k, sizeof k, "sensor%d_echo", n); sEcho[i] = cfgPin(k);
    snprintf(k, sizeof k, "sensor%d_analog", n); sAnalog[i] = cfgPin(k);
  }
  PID_KP = cfgF("pid_kp", 0); PID_KI = cfgF("pid_ki", 0); PID_KD = cfgF("pid_kd", 0);
  PID_V_MAX = cfgF("max_speed_mps", 0.2f); PID_V_MIN = min(max(0.0f, cfgF("v_min_mps", 0)), PID_V_MAX);
  PID_SPEED_FRAC = cfgF("pid_speed_frac", 0.5f); PID_ERROR_CLIP = cfgF("pid_error_clip_mm", 80) / 1000.0f;
  PID_INTEGRAL_LIMIT = cfgF("pid_integral_limit", 0.5f); PID_D_ALPHA = cfgF("pid_d_filter_alpha", 0.3f);
  float sl = fabs(sin(maxA * 3.14159265f / 180)), sr = fabs(sin(minA * 3.14159265f / 180));
  PID_SETPOINT = -(cfgF("pid_offset_mm", 0) / 1000.0f) * (1 / max(0.17f, sl) + 1 / max(0.17f, sr));
  configured = pinA[0] >= 0 && pinB[0] >= 0 && pinA[1] >= 0 && pinB[1] >= 0;
}

// called first by setup(): settings, Bluetooth, motor pins
void motorsSetup() {
  loadSettings();
  String bt = cfgGet("bluetooth", "yes");
  if (bt != "no") { String nm = cfgGet("bluetooth_name", "default"); if (nm == "default" || !nm.length()) nm = "RobotTuner"; btOn = (bool)SerialBT.begin(nm); }
  for (int s = 0; s < 2; s++) {
    if (pinA[s] >= 0) pinMode(pinA[s], OUTPUT); if (pinB[s] >= 0) pinMode(pinB[s], OUTPUT); if (drvType == 2 && pinEn[s] >= 0) pinMode(pinEn[s], OUTPUT);
  }
  if (pinStby >= 0) { pinMode(pinStby, OUTPUT); digitalWrite(pinStby, HIGH); }
  if (!configured) RT_IO.println(F("WARN not set up: send SET <key> <value> lines (Robot Tuner does it for you), then SAVE"));
}
void i2cSetup() { int sda = cfgPin("pin_i2c_sda"), scl = cfgPin("pin_i2c_scl"); Wire.begin(sda >= 0 ? sda : 21, scl >= 0 ? scl : 22); Wire.setClock(400000); }
void encSetup() {
  for (int s = 0; s < 2; s++) {
    if (encType == 2 || encA[s] < 0) continue;
    pinMode(encA[s], INPUT_PULLUP);
    if (encType == 0 && encB[s] >= 0) { pinMode(encB[s], INPUT_PULLUP); qState[s] = (digitalRead(encA[s]) << 1) | digitalRead(encB[s]);
      attachInterruptArg(digitalPinToInterrupt(encA[s]), encIsr, (void*)(intptr_t)s, CHANGE); attachInterruptArg(digitalPinToInterrupt(encB[s]), encIsr, (void*)(intptr_t)s, CHANGE); }
    else attachInterruptArg(digitalPinToInterrupt(encA[s]), encIsr, (void*)(intptr_t)s, RISING);
  }
}

// SET / GET / SAVE (u = upper case with "CAL " in front, orig = as typed)
bool extraCommand(const char* u, const char* orig) {
  if (strncmp(u, "CAL SET ", 8) == 0) {
    String s = String(orig + 4); s.trim(); int sp = s.indexOf(' ');
    if (sp < 1) { RT_IO.println(F("ERR SET needs a key and a value")); return true; }
    String key = s.substring(0, sp), val = s.substring(sp + 1); key.trim(); val.trim(); key.toLowerCase();
    if (val == "N/A") { cfgSet(key, ""); RT_IO.print(F("OK unset ")); RT_IO.println(key); return true; }
    cfgSet(key, val); RT_IO.print(F("OK SET ")); RT_IO.print(key); RT_IO.print(' '); RT_IO.println(val); return true;
  }
  if (strcmp(u, "CAL GET") == 0) {
    int p = 0; while (p < (int)cfgText.length()) { int e = cfgText.indexOf('\n', p); if (e < 0) e = cfgText.length(); String l = cfgText.substring(p, e); if (l.length()) { RT_IO.print(F("CFG ")); RT_IO.println(l); } p = e + 1; }
    RT_IO.print(F("CFG END ")); RT_IO.println(configured ? F("configured") : F("not set up")); return true;
  }
  if (strcmp(u, "CAL SAVE") == 0) {
    prefs.putString("cfg", cfgText); RT_IO.println(F("OK saved: restarting")); RT_IO.flush(); delay(300); ESP.restart(); return true;
  }
  if (strcmp(u, "CAL FACTORY") == 0) { prefs.clear(); RT_IO.println(F("OK settings erased: restarting")); RT_IO.flush(); delay(300); ESP.restart(); return true; }
  return false;
}
`;

/** the complete install sketch (USB_SERIAL_ = the hardware serial; every other Serial. call goes through RT_IO) */
export function generateEsp32Install() {
  const code = (PRELUDE() + '\n' + CORE)
    .replace(/\bSerial\.(print|println|available|read)\(/g, 'RT_IO.$1(')
    .replace(/USB_SERIAL_\./g, 'Serial.').replace('  BT_BEGIN\n', '');
  return { code, fileName: 'robot_tuner_esp32.ino', fqbn: 'esp32:esp32:esp32:PartitionScheme=huge_app' };
}
