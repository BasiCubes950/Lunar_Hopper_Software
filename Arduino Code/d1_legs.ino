// STAC Lunar Hopper - Leg Contact Sensing & Calibration
// Hardware: Arduino Nano, BNO055 IMU (I2C 0x28), 3x Actuonix L16-R Actuators

#include <Wire.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <math.h>

// Pins & Actuator Calibration
const uint8_t LEG_PINS[3] = {9, 10, 11};
const float STROKE_MM     = 100.0;
const float SPEED_MM_S    = 13.0;   // Measured extension speed (mm/s)
const int US_RETRACTED    = 1000;   // 0 mm extension
const int US_EXTENDED     = 2000;   // 100 mm extension

// Motion Thresholds
const float SETTLE_TILT_DEG  = 1.0;
const float SETTLE_RATE_RADS = 0.05;
const unsigned long SETTLE_MS = 500;
const float CONTACT_TILT_DEG = 2.0;
const float STEP_MM          = 1.0;
const float FOLD_RANGE_MM    = 20.0;

const unsigned long STEP_DELAY_MS = (unsigned long)(1000.0 * STEP_MM / SPEED_MM_S);

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Servo legServos[3];
float currentMm[3] = {0, 0, 0};
float contactMm[3] = {0, 0, 0};

void setLeg(uint8_t leg, float mm) {
  mm = constrain(mm, 0.0, STROKE_MM);
  currentMm[leg] = mm;
  int pulseWidth = US_RETRACTED + (int)((mm / STROKE_MM) * (US_EXTENDED - US_RETRACTED));
  legServos[leg].writeMicroseconds(pulseWidth);
}

float getTiltAngle(const imu::Vector<3>& v1, const imu::Vector<3>& v2) {
  float dot = v1.dot(v2) / (v1.magnitude() * v2.magnitude());
  return acos(constrain(dot, -1.0, 1.0)) * 180.0 / PI;
}

bool waitForStillness(unsigned long timeoutMs) {
  unsigned long tStart = millis();
  unsigned long tStableStart = 0;
  imu::Vector<3> refGravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

  while (millis() - tStart < timeoutMs) {
    imu::Vector<3> g = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    float gyroMag = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE).magnitude();

    bool isStill = (getTiltAngle(g, refGravity) < SETTLE_TILT_DEG) && (gyroMag < SETTLE_RATE_RADS);

    if (!isStill) {
      refGravity = g;
      tStableStart = 0;
    } else {
      if (tStableStart == 0) tStableStart = millis();
      if (millis() - tStableStart >= SETTLE_MS) return true;
    }
    delay(20);
  }
  return false;
}

bool probeLeg(uint8_t legIndex) {
  if (!waitForStillness(3000)) return false;

  imu::Vector<3> baselineGravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

  for (float mm = currentMm[legIndex]; mm <= STROKE_MM; mm += STEP_MM) {
    setLeg(legIndex, mm);
    delay(STEP_DELAY_MS);

    imu::Vector<3> currentGravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    if (getTiltAngle(currentGravity, baselineGravity) > CONTACT_TILT_DEG) {
      contactMm[legIndex] = mm;
      return true;
    }
  }
  return false;
}

void setup() {
  Serial.begin(115200);

  if (!bno.begin()) {
    Serial.println(F("ERROR: BNO055 initialization failed. Check I2C lines."));
    while (1);
  }
  bno.setExtCrystalUse(true);

  for (uint8_t i = 0; i < 3; i++) {
    legServos[i].attach(LEG_PINS[i]);
    setLeg(i, 0);
  }

  // Allow physical retraction before calibration
  delay((unsigned long)(1000.0 * STROKE_MM / SPEED_MM_S) + 500);
}

void loop() {
  for (uint8_t i = 0; i < 3; i++) {
    if (probeLeg(i)) {
      Serial.print(F("Leg "));
      Serial.print(i);
      Serial.print(F(" ground contact detected at: "));
      Serial.print(contactMm[i]);
      Serial.println(F(" mm"));
    } else {
      Serial.print(F("Leg "));
      Serial.print(i);
      Serial.println(F(" probe failed / reached full extension."));
    }
  }

  // Hold contact positions indefinitely
  for (uint8_t i = 0; i < 3; i++) setLeg(i, contactMm[i]);
  while (1);
}
