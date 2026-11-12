// STAC Lunar Hopper - Continuous Leg Oscillation Test (NO IMU)
// Hardware: Arduino Nano, 2x Actuonix L16-R Actuators (pins 9 and 10)
// Purpose: endlessly sweep the legs back and forth.

#include <Servo.h>

// Pins & Actuator Calibration
const uint8_t NUM_LEGS    = 2;
const uint8_t LEG_PINS[NUM_LEGS] = {9, 10};
const float STROKE_MM     = 100.0;
const float SPEED_MM_S    = 13.0;   // Estimated extension speed (mm/s)
const int US_RETRACTED    = 1000;   // 0 mm extension
const int US_EXTENDED     = 2000;   // 100 mm extension

// Oscillation settings
const float LOW_MM   = 10.0;        // Lower end of the sweep
const float HIGH_MM  = 90.0;        // Upper end of the sweep
const unsigned long HOLD_MS = 300;  // Pause at each end

// Time to travel the sweep range, plus margin
const unsigned long SWEEP_MS = (unsigned long)(1000.0 * (HIGH_MM - LOW_MM) / SPEED_MM_S) + 300;

Servo legServos[NUM_LEGS];

void setLeg(uint8_t leg, float mm) {
  mm = constrain(mm, 0.0, STROKE_MM);
  int pulseWidth = US_RETRACTED + (int)((mm / STROKE_MM) * (US_EXTENDED - US_RETRACTED));
  legServos[leg].writeMicroseconds(pulseWidth);
}

void setAll(float mm) {
  for (uint8_t i = 0; i < NUM_LEGS; i++) setLeg(i, mm);
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("Leg oscillation test starting (no IMU)."));

  for (uint8_t i = 0; i < NUM_LEGS; i++) legServos[i].attach(LEG_PINS[i]);

  setAll(LOW_MM);
  delay((unsigned long)(1000.0 * LOW_MM / SPEED_MM_S) + 1000);
}

void loop() {
  Serial.println(F("Extending"));
  setAll(HIGH_MM);
  delay(SWEEP_MS + HOLD_MS);

  Serial.println(F("Retracting"));
  setAll(LOW_MM);
  delay(SWEEP_MS + HOLD_MS);
}
