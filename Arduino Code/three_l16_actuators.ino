// Standalone three-actuator test: open it on its own to use it. When compiled together with the
// main program (HopperLegs.ino defines HOPPER_MAIN_SKETCH) this whole file is skipped.
#ifndef HOPPER_MAIN_SKETCH

#include <Servo.h>

// ---------- Configuration ----------
const uint8_t NUM_ACTUATORS = 3;
const uint8_t ACT_PINS[NUM_ACTUATORS] = {9, 10, 11};    // PUT IN ACTUAL PINS!!!!!

const float STROKE_MM    = 100.0;  // L16-100 max range
const float MAX_SPEED_MM = 13.0;   // mm/s, ~no-load speed for 150:1; measure yours
// -----------------------------------

class LinearServo {
  public:
    void begin(uint8_t pin, float length, float speed) {
      m_length = length;
      m_speed  = speed;
      m_servo.attach(pin, 1000, 2000);
      m_curPos = m_length;          // unknown start: assume fully extended
      m_lastMs = millis();
      setPosition(0);               // home to retracted
    }

    void setPosition(float setpoint) {
      m_setPos = constrain(setpoint, 0.0f, m_length);
      int us = (int)(1000.0f + (m_setPos / m_length) * 1000.0f);
      m_servo.writeMicroseconds(us);
    }

    void update() {
      unsigned long now = millis();
      float step = m_speed * (now - m_lastMs) / 1000.0f;
      m_lastMs = now;

      if (m_curPos > m_setPos + step)      m_curPos -= step;
      else if (m_curPos < m_setPos - step) m_curPos += step;
      else                                 m_curPos = m_setPos;
    }

    float getPosition() const { return m_curPos; }
    float getTarget()   const { return m_setPos; }
    bool  isFinished()  const { return m_curPos == m_setPos; }

  private:
    Servo m_servo;
    float m_length = 100.0f;
    float m_speed  = 13.0f;
    float m_setPos = 0.0f;
    float m_curPos = 0.0f;
    unsigned long m_lastMs = 0;
};

LinearServo actuators[NUM_ACTUATORS];

// ---------------- Public control functions ----------------

// Move actuator `index` to an absolute position (ranges from 0-100mm)
void moveActuatorTo(uint8_t index, float positionMm) {
  if (index >= NUM_ACTUATORS) return;
  actuators[index].setPosition(positionMm);
}

// Move actuator `index` by `deltaMm`
void moveActuatorBy(uint8_t index, float deltaMm) {
  if (index >= NUM_ACTUATORS) return;
  actuators[index].setPosition(actuators[index].getTarget() + deltaMm);
}

// Get function for position
float getActuatorPosition(uint8_t index) {
  if (index >= NUM_ACTUATORS) return -1;
  return actuators[index].getPosition();
}


bool actuatorIsDone(uint8_t index) {
  if (index >= NUM_ACTUATORS) return true;
  return actuators[index].isFinished();
}

// Runs once per loop
void updateActuators() {
  for (uint8_t i = 0; i < NUM_ACTUATORS; i++) actuators[i].update();
}

// ---------------- Demo ----------------
// Demo to show each index has both functions working
uint8_t demoStep = 0;
unsigned long demoWaitStart = 0;

void setup() {
  Serial.begin(115200);   // IF THE CODE DOESN'T WORK ADJUST THIS VALUE
  // 300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, and 115200
  for (uint8_t i = 0; i < NUM_ACTUATORS; i++) {
    actuators[i].begin(ACT_PINS[i], STROKE_MM, MAX_SPEED_MM);
  }
}

void loop() {
  updateActuators();   // must run every loop; avoid long delay() calls

  bool allDone = actuatorIsDone(0) && actuatorIsDone(1) && actuatorIsDone(2);
  if (allDone) {
    if (demoWaitStart == 0) demoWaitStart = millis();
    if (millis() - demoWaitStart > 1000) {
      demoWaitStart = 0;
      switch (demoStep) {
        case 0: moveActuatorTo(0, 50);                       break; // index 0 to 50 mm
        case 1: moveActuatorTo(1, 30);                       break; // index 1 to 30 mm
        case 2: moveActuatorTo(2, 80);                       break; // index 1 to 80 mm
        case 3: moveActuatorBy(0, -10);                      break; // index 0 back 10 mm
        case 4: moveActuatorBy(1, 15);                       break; // index 1 forward 15 mm
        case 5: moveActuatorBy(2, -20);                      break; // index 2 back 20 mm
        case 6: moveActuatorTo(0, 0);
                moveActuatorTo(1, 0);
                moveActuatorTo(2, 0);                        break; // all home
      }
      demoStep = (demoStep + 1) % 7;
    }
  }
}

#endif  // HOPPER_MAIN_SKETCH
