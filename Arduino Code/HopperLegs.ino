/*
 * Lunar Hopper - legs main sketch (bench-test version)
 * Tabs: leg_control_d1 (leg actuators), self_right_d2 (self-righting)
 *
 * Self-righting starts by itself when the hopper is still and tilted.
 *
 * Serial monitor at 115200:
 *   r = start self-righting by hand      f = fold legs test (40 mm lowered)
 *   h = legs to launch pose      0 = legs home
 */

uint8_t lastState = 255;

void setup() {
  Serial.begin(115200);
  beginLegs();
  if (!selfRightingBegin()) Serial.println("BNO055 not found - check wiring");
}

void loop() {
  updateLegs();            // must run every loop; avoid long delay() calls
  checkAutoSelfRighting();   // starts D2 when still + tilted
  updateSelfRighting();

  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'r') startSelfRighting();
    if (c == 'f') foldLegsForJump(40.0f);
    if (c == 'h') for (uint8_t i = 0; i < NUM_LEGS; i++) moveLegTo(i, LAUNCH_POSE_MM);
    if (c == '0') for (uint8_t i = 0; i < NUM_LEGS; i++) moveLegTo(i, 0);
  }

  uint8_t s = selfRightingState();
  if (s != lastState) {
    lastState = s;
    Serial.print("self-right state: "); Serial.println(s);   // 0 idle,1 wait,2 decide,3 moving,4 finishing,5 done,6 failed
  }
}
