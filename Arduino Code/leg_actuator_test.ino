// STAC Lunar Hopper - Leg Actuator Movement Test (NO IMU)
// Hardware: Arduino Nano, 3x Actuonix L16-R Actuators
// Purpose: verify each actuator physically moves. No BNO055 needed.

#include <Servo.h>

// Pins & Actuator Calibration
const uint8_t LEG_PINS[3] = {9, 10, 11};
const float STROKE_MM     = 100.0;
const float SPEED_MM_S    = 13.0;   // Estimated extension speed (mm/s)
const int US_RETRACTED    = 1000;   // 0 mm extension
const int US_EXTENDED     = 2000;   // 100 mm extension

// Test settings
const float TEST_MM = 50.0;         // How far to extend during the test (keep modest at first)

// Time to travel the full stroke, plus margin
const unsigned long FULL_TRAVEL_MS = (unsigned long)(1000.0 * STROKE_MM / SPEED_MM_S) + 500;

Servo legServos[3];
float currentMm[3] = {0, 0, 0};

void setLeg(uint8_t leg, float mm) {
  mm = constrain(mm, 0.0, STROKE_MM);
  currentMm[leg] = mm;
  int pulseWidth = US_RETRACTED + (int)((mm / STROKE_MM) * (US_EXTENDED - US_RETRACTED));
  legServos[leg].writeMicroseconds(pulseWidth);
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("Leg actuator test starting (no IMU)."));

  for (uint8_t i = 0; i < 3; i++) {
    legServos[i].attach(LEG_PINS[i]);
    setLeg(i, 0);
  }

  Serial.println(F("Retracting all legs..."));
  delay(FULL_TRAVEL_MS);
}

void loop() {
  // Test each leg one at a time: extend, hold, retract
  for (uint8_t i = 0; i < 3; i++) {
    Serial.print(F("Leg "));
    Serial.print(i);
    Serial.print(F(" (pin "));
    Serial.print(LEG_PINS[i]);
    Serial.print(F(") extending to "));
    Serial.print(TEST_MM);
    Serial.println(F(" mm"));
    setLeg(i, TEST_MM);
    delay((unsigned long)(1000.0 * TEST_MM / SPEED_MM_S) + 500);

    delay(1000);  // hold

    Serial.print(F("Leg "));
    Serial.print(i);
    Serial.println(F(" retracting"));
    setLeg(i, 0);
    delay((unsigned long)(1000.0 * TEST_MM / SPEED_MM_S) + 500);

    delay(500);
  }

  // Then all three together
  Serial.println(F("All legs extending together"));
  for (uint8_t i = 0; i < 3; i++) setLeg(i, TEST_MM);
  delay((unsigned long)(1000.0 * TEST_MM / SPEED_MM_S) + 1000);

  Serial.println(F("All legs retracting together"));
  for (uint8_t i = 0; i < 3; i++) setLeg(i, 0);
  delay(FULL_TRAVEL_MS);

  Serial.println(F("Test cycle done. Holding retracted."));
  while (1);  // run once; press reset to repeat
}
