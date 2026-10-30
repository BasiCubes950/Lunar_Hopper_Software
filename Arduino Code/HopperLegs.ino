// Marks the full hopper program. IMU.ino and three_l16_actuators.ino are standalone tests:
// they see this and switch themselves off, so all files can sit in one folder.
#define HOPPER_MAIN_SKETCH

// Defined in LegActuators.ino, which Arduino compiles AFTER this file.
extern const uint8_t NUM_LEGS;
extern const float   LAUNCH_POSE_MM;

uint8_t lastState = 255;
bool imuFaultHandled = false;

void setup() {
  Serial.begin(115200);
  beginLegs();
  if (!imuBegin()) Serial.println("BNO055 not found - check wiring. Legs only home, no other moves.");   // D5
  imuFaultHandled = imuHasFault();   // at boot the legs just finish homing (position not known yet)
}

void loop() {
  imuUpdate();             // D5: reads the IMU on its own timer, never blocks
  updateLegs();            // must run every loop; avoid long delay() calls
  if (imuHasFault()) {     // D5: IMU missing/lost -> stop the legs once, skip everything else
    if (!imuFaultHandled) {
      imuFaultHandled = true;
      for (uint8_t i = 0; i < NUM_LEGS; i++) moveLegTo(i, getLegPosition(i));
      Serial.println("IMU FAULT - legs stopped");
    }
    return;
  }
  checkAutoSelfRighting();   // starts D2 when still + tilted
  updateSelfRighting();

  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'r') startSelfRighting();
    if (c == 'f') foldLegsForJump(40.0f);
    if (c == 'h') for (uint8_t i = 0; i < NUM_LEGS; i++) moveLegTo(i, LAUNCH_POSE_MM);
    if (c == '0') for (uint8_t i = 0; i < NUM_LEGS; i++) moveLegTo(i, 0);
    imuSimCommand(c);      // D5: U T S D M F L = pretend IMU data (ignored with the real IMU)
  }

  uint8_t s = selfRightingState();
  if (s != lastState) {
    lastState = s;
    Serial.print("self-right state: "); Serial.println(s);   // 0 idle,1 wait,2 decide,3 moving,4 finishing,5 done,6 failed
  }
}
