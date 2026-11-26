/*
  MediX biceps-curl tracker firmware v2  (XIAO ESP32-S3 Plus, Arduino-ESP32 core 3.x)
  IMU (MPU6050) is strapped on the BICEPS (upper arm). Upper arm must stay still and vertical.
  REPS are counted from biceps EMG contraction (not from IMU angle).
  Calibration "C" (3 steps x 3 s, short beep at each step):
    1) arm hanging straight down, relaxed   -> rest EMG + vertical reference
    2) raise straight arm forward to shoulder height -> learns the "forward" direction
    3) flex biceps as hard as you can       -> 100% biceps reference
  BLE packet, 12 bytes little-endian: u16 env1(biceps), u16 env2(shoulder), i16 fwdDev*10, i16 latDev*10, u8 reps, u8 state, u8 warn, u8 seq
    fwdDev: + = upper arm tilted FORWARD, - = backward (deg).  latDev: + = tilted to the person's LEFT, - = RIGHT (deg).
    state: 0 idle, 1/4/5 calibration steps, 2 active, 3 target reached
    warn bits: 1 shoulder compensating, 2 angle error (upper arm off vertical), 4 rep rejected, 8 direction unknown (step 2 failed)
  Commands: "C" calibrate | "S12" start set, 12 reps | "X" stop | "H30" shoulder threshold 3.0 x rest
  PROTOTYPE thresholds: tune on a real arm.
*/
#include <Wire.h>
#include <math.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

// ---- PIN MAP (GPIO numbers). D3=4 D4=5 D5=6 D8=7 D9=8 D10=9
#define PIN_EMG1 8   // biceps   = D9
#define PIN_EMG2 9   // shoulder = D10
#define PIN_BUZ  4   // buzzer   = D3
#define PIN_SDA  5   // IMU SDA  = D4
#define PIN_SCL  6   // IMU SCL  = D5
#define BUZZER_ACTIVE 0          // 1 = active buzzer (DC on/off), 0 = passive buzzer (tone)

#define SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define RX_UUID      "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define TX_UUID      "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

// ---- tunables
const float DEV_LIMIT = 15;                                 // deg off vertical before "angle error"
const float HI_FRAC = 0.35f, LO_FRAC = 0.15f;               // rep thresholds as fraction of (flex - rest)
const uint32_t CAL_MS = 3000, DEV_HOLD_MS = 300, SH_HOLD_MS = 150, MIN_REP_MS = 600, COOLDOWN_MS = 400;
float shMult = 3.0f; const float SH_ABS = 15;              // shoulder limit = rest*mult + abs (ADC counts)

// ---- state
BLECharacteristic *txChar; bool connected = false;
uint8_t state = 0, reps = 0, target = 12, seq = 0, warn = 0;
float env1 = 0, env2 = 0, dc1 = 2000, dc2 = 2000, rest1 = 0, rest2 = 0, mvc1 = 100;
float m0[3] = {0, 0, 1}, fwd[3] = {1, 0, 0}, lat[3] = {0, 1, 0}, gLP[3] = {0, 0, 1};
bool dirOK = false; float thF = 0, thL = 0;
uint8_t repPhase = 0; bool repBad = false; uint32_t repStart = 0, lastEnd = 0, rejectUntil = 0, shSince = 0, devSince = 0;
uint32_t calStart = 0, nCal = 0; float cA[3], cB[3], cr1, cr2, mvcPeak;

// ---- vector helpers
float dot3(const float *a, const float *b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
void norm3(float *a) { float n = sqrtf(dot3(a, a)); if (n < 1e-6f) n = 1e-6f; a[0] /= n; a[1] /= n; a[2] /= n; }
void cross3(const float *a, const float *b, float *o) { o[0] = a[1] * b[2] - a[2] * b[1]; o[1] = a[2] * b[0] - a[0] * b[2]; o[2] = a[0] * b[1] - a[1] * b[0]; }

// ---- buzzer (non-blocking)
struct Beep { uint16_t f, ms; };
Beep q[6]; uint8_t qn = 0, qi = 0; uint32_t qt = 0; bool qrun = false;
void setTone(uint16_t f) {
#if BUZZER_ACTIVE
  digitalWrite(PIN_BUZ, f ? HIGH : LOW);
#else
  ledcWriteTone(PIN_BUZ, f);
#endif
}
void play(const Beep *b, uint8_t n) { for (uint8_t i = 0; i < n; i++) q[i] = b[i]; qn = n; qi = 0; qrun = true; qt = millis(); setTone(q[0].f); }
void beepUpdate() {
  if (!qrun || millis() - qt < q[qi].ms) return;
  qi++; qt = millis();
  if (qi >= qn) { qrun = false; setTone(0); } else setTone(q[qi].f);
}
void bStep()     { Beep b[] = {{1800, 90}}; play(b, 1); }
void bRep()      { Beep b[] = {{2500, 50}}; play(b, 1); }
void bShoulder() { Beep b[] = {{1200, 120}, {0, 80}, {1200, 120}}; play(b, 3); }
void bAngle()    { Beep b[] = {{2000, 450}}; play(b, 1); }
void bDone()     { Beep b[] = {{2000, 100}, {0, 60}, {2500, 100}, {0, 60}, {3000, 200}}; play(b, 5); }

// ---- BLE
class SrvCB : public BLEServerCallbacks {
  void onConnect(BLEServer *) { connected = true; }
  void onDisconnect(BLEServer *s) { connected = false; s->startAdvertising(); }
};
class RxCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) {
    auto v = c->getValue(); const char *s = v.c_str(); if (!*s) return;
    if (s[0] == 'C') { state = 1; calStart = millis(); nCal = 0; for (int i = 0; i < 3; i++) cA[i] = cB[i] = 0; cr1 = cr2 = 0; bStep(); }
    else if (s[0] == 'S') { target = max(1, atoi(s + 1)); reps = 0; repPhase = 0; repBad = false; state = 2; }
    else if (s[0] == 'X') { state = 0; }
    else if (s[0] == 'H') { float m = atoi(s + 1) / 10.0f; if (m >= 1 && m <= 10) shMult = m; }
  }
};

// ---- MPU6050
void mpuWrite(uint8_t r, uint8_t v) { Wire.beginTransmission(0x68); Wire.write(r); Wire.write(v); Wire.endTransmission(); }
bool mpuAccel(float a[3]) {
  Wire.beginTransmission(0x68); Wire.write(0x3B); if (Wire.endTransmission(false)) return false;
  if (Wire.requestFrom(0x68, 6) != 6) return false;
  for (int i = 0; i < 3; i++) { int hi = Wire.read(); int lo = Wire.read(); a[i] = (int16_t)((hi << 8) | lo) / 16384.0f; }
  return true;
}

void setup() {
  pinMode(PIN_BUZ, OUTPUT);
#if !BUZZER_ACTIVE
  ledcAttach(PIN_BUZ, 2000, 8);
#endif
  analogReadResolution(12); analogSetAttenuation(ADC_11db);
  Wire.begin(PIN_SDA, PIN_SCL); Wire.setClock(400000);
  mpuWrite(0x6B, 0); mpuWrite(0x1A, 3); mpuWrite(0x1C, 0);
  BLEDevice::init("MediX");
  BLEServer *srv = BLEDevice::createServer(); srv->setCallbacks(new SrvCB());
  BLEService *svc = srv->createService(SERVICE_UUID);
  txChar = svc->createCharacteristic(TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  BLECharacteristic *rx = svc->createCharacteristic(RX_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rx->setCallbacks(new RxCB()); svc->start();
  BLEAdvertising *adv = BLEDevice::getAdvertising(); adv->addServiceUUID(SERVICE_UUID); adv->setScanResponse(true); adv->start();
}

// rep = one biceps contraction burst (above HI) that returns below LO
void repLogic(uint32_t now, bool shFlag) {
  if (state != 2) return;
  float span = max(mvc1 - rest1, 20.0f), hi = rest1 + HI_FRAC * span, lo = rest1 + LO_FRAC * span;
  if (repPhase == 0) {
    if (env1 > hi && now - lastEnd > COOLDOWN_MS) { repPhase = 1; repStart = now; repBad = false; }
  } else {
    if (shFlag) repBad = true;
    if (env1 < lo) {
      repPhase = 0; lastEnd = now;
      if (now - repStart < MIN_REP_MS) return;                 // too short: ignore
      if (repBad) { rejectUntil = now + 1500; return; }        // shoulder cheated: not counted
      reps++;
      if (reps >= target) { state = 3; bDone(); } else bRep();
    }
  }
}

void calibrate(uint32_t now, const float *u) {
  uint32_t el = now - calStart;
  if (state == 1) {
    for (int i = 0; i < 3; i++) cA[i] += u[i]; cr1 += env1; cr2 += env2; nCal++;
    if (el >= CAL_MS && nCal) {
      for (int i = 0; i < 3; i++) m0[i] = cA[i] / nCal; norm3(m0); rest1 = cr1 / nCal; rest2 = cr2 / nCal;
      state = 4; calStart = now; nCal = 0; bStep();
    }
  } else if (state == 4) {
    for (int i = 0; i < 3; i++) cB[i] += u[i]; nCal++;
    if (el >= CAL_MS && nCal) {
      float u1[3] = {cB[0] / nCal, cB[1] / nCal, cB[2] / nCal}; norm3(u1);
      float k = dot3(u1, m0), d[3] = {u1[0] - k * m0[0], u1[1] - k * m0[1], u1[2] - k * m0[2]};
      float len = sqrtf(dot3(d, d));
      if (len > 0.5f) { for (int i = 0; i < 3; i++) fwd[i] = d[i] / len; dirOK = true; }
      else { dirOK = false; float ax[3] = {fabsf(m0[0]) < 0.8f ? 1.0f : 0.0f, fabsf(m0[0]) < 0.8f ? 0.0f : 1.0f, 0}; float kk = dot3(ax, m0); for (int i = 0; i < 3; i++) fwd[i] = ax[i] - kk * m0[i]; norm3(fwd); }
      cross3(m0, fwd, lat); norm3(lat);                        // lat = m0 x fwd = person's LEFT
      state = 5; calStart = now; mvcPeak = 0; bStep();
    }
  } else if (state == 5) {
    mvcPeak = max(mvcPeak, env1);
    if (el >= CAL_MS) { mvc1 = max(mvcPeak, rest1 + 20); state = 0; bDone(); }
  }
}

void loop() {
  static uint32_t t1 = 0, t10 = 0, t50 = 0;
  uint32_t now = millis(), us = micros();
  if (us - t1 >= 1000) {                                       // EMG @ 1 kHz: remove DC, rectify, ~100 ms smoothing
    t1 = us;
    float x1 = analogRead(PIN_EMG1), x2 = analogRead(PIN_EMG2);
    dc1 += (x1 - dc1) * 0.001f; dc2 += (x2 - dc2) * 0.001f;
    env1 += (fabsf(x1 - dc1) - env1) * 0.01f; env2 += (fabsf(x2 - dc2) - env2) * 0.01f;
  }
  if (now - t10 >= 10) {                                       // IMU + logic @ 100 Hz
    t10 = now; float a[3];
    if (mpuAccel(a)) {
      for (int i = 0; i < 3; i++) gLP[i] += (a[i] - gLP[i]) * 0.2f;
      float u[3] = {gLP[0], gLP[1], gLP[2]}; norm3(u);
      if (state == 1 || state == 4 || state == 5) calibrate(now, u);
      thF = asinf(constrain(dot3(u, fwd), -1.0f, 1.0f)) * 57.2958f;
      thL = asinf(constrain(dot3(u, lat), -1.0f, 1.0f)) * 57.2958f;
      float dev = sqrtf(thF * thF + thL * thL);
      bool sh = (state == 2) && (env2 > rest2 * shMult + SH_ABS);
      if (sh) { if (!shSince) shSince = now; } else shSince = 0;
      bool shFlag = sh && (now - shSince > SH_HOLD_MS);
      bool dv = (state == 2) && dev > DEV_LIMIT;
      if (dv) { if (!devSince) devSince = now; } else devSince = 0;
      bool devFlag = dv && (now - devSince > DEV_HOLD_MS);
      static bool pSh = false, pDv = false;
      if (shFlag && !pSh) bShoulder(); else if (devFlag && !pDv && !shFlag) bAngle();
      pSh = shFlag; pDv = devFlag;
      warn = (shFlag ? 1 : 0) | (devFlag ? 2 : 0) | (now < rejectUntil ? 4 : 0) | (dirOK ? 0 : 8);
      repLogic(now, shFlag);
    }
  }
  if (connected && now - t50 >= 50) {                          // BLE notify @ 20 Hz
    t50 = now; uint8_t p[12]; uint16_t e1 = min(env1, 65535.0f), e2 = min(env2, 65535.0f);
    int16_t f10 = thF * 10, l10 = thL * 10;
    memcpy(p, &e1, 2); memcpy(p + 2, &e2, 2); memcpy(p + 4, &f10, 2); memcpy(p + 6, &l10, 2);
    p[8] = reps; p[9] = state; p[10] = warn; p[11] = seq++;
    txChar->setValue(p, 12); txChar->notify();
  }
  beepUpdate();
}
