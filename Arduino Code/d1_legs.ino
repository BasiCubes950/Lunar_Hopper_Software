// D1: STAC lunar hopper leg control (SKETCH, not yet compiled or tested on hardware)
//
// Hardware: Arduino Nano, Adafruit BNO055 (I2C: A4 = SDA, A5 = SCL), 3x Actuonix L16-R.
// The L16-R takes an RC-servo signal (1000-2000 us) and has NO position feedback, so leg
// position is an open-loop estimate. That is why ground contact is detected from the IMU.
//
// Power the actuators from a regulated supply (buck converter), not the Nano's 5V pin,
// and share a common ground with the Nano.
//
// Contact detection, per leg (one leg at a time):
//   1. Wait until the hopper is still (gravity vector steady and gyro near zero).
//   2. Capture the baseline gravity vector.
//   3. Extend that leg in small steps, re-reading the IMU after each step.
//   4. When the gravity vector has rotated past CONTACT_TILT_DEG from the baseline,
//      the leg is bearing weight: stop and record its position.
// Tilt is measured relative to the baseline, so the BNO055 mounting axes don't matter.

#include <Wire.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <math.h>

// ---------- Config (TODO: confirm/measure) ----------
const uint8_t LEG_PINS[3]  = {9, 10, 11};
const float STROKE_MM      = 100.0;  // TODO: confirm stroke of the ordered L16-R
const float SPEED_MM_S     = 13.0;   // ESTIMATE: measure in person
const int   US_RETRACTED   = 1000;   // L16-R: ~1000 us = retracted, ~2000 us = extended
const int   US_EXTENDED    = 2000;   // TODO: confirm against datasheet / calibrate

const float SETTLE_TILT_DEG  = 1.0;    // max gravity-vector drift while "still"
const float SETTLE_RATE_RADS = 0.05;   // max gyro magnitude while "still" (rad/s)
const unsigned long SETTLE_MS = 500;   // how long it must stay still
const float CONTACT_TILT_DEG = 2.0;    // tilt from baseline that means "touching ground"
const float STEP_MM          = 1.0;    // extension per probe step
const float FOLD_RANGE_MM    = 20.0;   // TODO: how far legs fold at full compression

// Wait for the leg to physically move one step before reading the IMU
const unsigned long STEP_DELAY_MS = (unsigned long)(1000.0 * STEP_MM / SPEED_MM_S);

// ---------- State ----------
struct Leg {
  Servo servo;
  float posMm = 0;  // commanded position; the real leg lags behind this

  void attach(uint8_t pin) {
    servo.attach(pin);
    write(0);
  }

  void write(float mm) {
    mm = constrain(mm, 0.0, STROKE_MM);
    posMm = mm;
    servo.writeMicroseconds((int)(US_RETRACTED + (mm / STROKE_MM) * (US_EXTENDED - US_RETRACTED)));
  }
};

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Leg legs[3];
float contactMm[3] = {0, 0, 0};   // extension at which each leg touched ground

// ---------- IMU helpers ----------
imu::Vector<3> readGravity() {
  return bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
}

float gyroMagnitude() {
  // VECTOR_GYROSCOPE is rad/s in recent Adafruit_BNO055 versions; check your version
  return bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE).magnitude();
}

float angleBetweenDeg(const imu::Vector<3>& a, const imu::Vector<3>& b) {
  float c = a.dot(b) / (a.magnitude() * b.magnitude());
  c = constrain(c, -1.0, 1.0);
  return acos(c) * 180.0 / PI;
}

// ---------- D1 behaviors ----------

// Block until the hopper has been still for SETTLE_MS, or give up after timeoutMs.
bool waitUntilSettled(unsigned long timeoutMs) {
  unsigned long start = millis();
  unsigned long stableSince = 0;
  imu::Vector<3> ref = readGravity();

  while (millis() - start < timeoutMs) {
    imu::Vector<3> g = readGravity();
    bool still = angleBetweenDeg(g, ref) < SETTLE_TILT_DEG && gyroMagnitude() < SETTLE_RATE_RADS;

    if (!still) {
      ref = g;
      stableSince = 0;
    } else {
      if (stableSince == 0) stableSince = millis();
      if (millis() - stableSince >= SETTLE_MS) return true;
    }
    delay(20);
  }
  return false;
}

// Extend one leg until the IMU shows it is bearing weight.
// Returns true and sets contactMm[i] on contact; false if never settled or never touched.
bool probeLeg(uint8_t i) {
  if (!waitUntilSettled(3000)) return false;

  imu::Vector<3> baseline = readGravity();

  for (float mm = legs[i].posMm; mm <= STROKE_MM; mm += STEP_MM) {
    legs[i].write(mm);
    delay(STEP_DELAY_MS);

    if (angleBetweenDeg(readGravity(), baseline) > CONTACT_TILT_DEG) {
      contactMm[i] = mm;   // leg is on the ground; it stays here
      return true;
    }
  }
  return false;  // fully extended without touching anything
}

// Hold every leg at its ground-contact length (straight legs for the jump).
void holdStraight() {
  for (uint8_t i = 0; i < 3; i++) legs[i].write(contactMm[i]);
}

// Fold legs slightly as the hopper lowers so they stay on the ground
// without blocking the carbon fibers.
// compression: 0 = not lowered, 1 = fully lowered.
// TODO: nothing supplies this value yet.
void foldLegs(float compression) {
  compression = constrain(compression, 0.0, 1.0);
  for (uint8_t i = 0; i < 3; i++) legs[i].write(contactMm[i] - compression * FOLD_RANGE_MM);
}

// ---------- Arduino entry points ----------
void setup() {
  Serial.begin(115200);

  if (!bno.begin()) {
    Serial.println(F("BNO055 not found, check wiring"));
    while (1) {}
  }
  bno.setExtCrystalUse(true);

  for (uint8_t i = 0; i < 3; i++) legs[i].attach(LEG_PINS[i]);

  // Give the legs time to reach full retraction before doing anything
  delay((unsigned long)(1000.0 * STROKE_MM / SPEED_MM_S) + 500);
}

void loop() {
  // Demo: probe each leg in turn, one at a time, and report the results.
  for (uint8_t i = 0; i < 3; i++) {
    bool found = probeLeg(i);
    Serial.print(F("Leg "));
    Serial.print(i);
    if (found) {
      Serial.print(F(" contact at "));
      Serial.print(contactMm[i]);
      Serial.println(F(" mm"));
    } else {
      Serial.println(F(" no contact (not settled, or fully extended)"));
    }
  }

  holdStraight();
  while (1) {}
}
