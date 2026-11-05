/*
  STAC lunar hopper: D6 MAIN CONTROLLER (SIMULATION ONLY)
  Runs through 11 steps using pretend sensors and actuators.
  NO REAL MOTORS OR LATCHES ARE CONTROLLED BY THIS SKETCH.
  Do not connect an actual release mechanism to this prototype.

  Commands in Serial Monitor (115200 baud):
    s = emergency stop    r = manually restart from SETTLE
    t = hold this step until it times out (for testing)
    n = stop holding steps
    b = simulate battery failure   g = restore pretend battery
    i = simulate IMU failure       k = restore pretend IMU
    u = simulate tilted hopper     o = restore upright (pretend)
*/

const byte STOP_BUTTON_PIN = 2;   // Button between pin 2 and GND
const unsigned long STEP_DURATION_MS = 800;  // SIMULATION ONLY
const unsigned long STEP_TIMEOUT_MS  = 3000; // SIMULATION ONLY

enum State {
  SETTLE, WIND, LATCH, BACK_OFF, FOLD_LEGS, READY,
  RELEASE, FLYING, LANDED, STAND_UP, GET_READY_AGAIN,
  EMERGENCY_STOP, FAULT
};

State currentState = SETTLE;
unsigned long stateStartedAt = 0;

#include "SimulatedSubsystems.h"

// Disable all MOCK actuators. Real hardware needs independent safety protection.
void safeStopAll() {
  Legs::stop();
  Winch::stop();
  Latch::inhibitRelease();
  IMU::stop();
}

const char* stateName(State state) {
  switch (state) {
    case SETTLE:          return "SETTLE";
    case WIND:            return "WIND";
    case LATCH:           return "LATCH";
    case BACK_OFF:        return "BACK_OFF";
    case FOLD_LEGS:       return "FOLD_LEGS";
    case READY:           return "READY";
    case RELEASE:         return "RELEASE (MOCK ONLY)";
    case FLYING:          return "FLYING";
    case LANDED:          return "LANDED";
    case STAND_UP:        return "STAND_UP";
    case GET_READY_AGAIN: return "GET_READY_AGAIN";
    case EMERGENCY_STOP:  return "EMERGENCY_STOP";
    case FAULT:           return "FAULT";
  }
  return "UNKNOWN";
}

void startStateOperation(State state);
bool safeToRelease();
void enterFault(const char* reason);

void changeState(State next) {
  currentState = next;
  stateStartedAt = millis();
  Serial.print("STATE -> ");
  Serial.println(stateName(next));
  startStateOperation(next);
}

void emergencyStop() {
  if (currentState == EMERGENCY_STOP || currentState == FAULT) return;
  safeStopAll();
  changeState(EMERGENCY_STOP);
  Serial.println("STOPPED: manual restart required");
}

void enterFault(const char* reason) {
  if (currentState == FAULT || currentState == EMERGENCY_STOP) return;
  safeStopAll();
  changeState(FAULT);
  Serial.print("FAULT: ");
  Serial.println(reason);
}

// These are release interlocks to TEST in a simulator.
// They are NOT sufficient as a real-world safety system.
bool safeToRelease() {
  return Battery::isHealthy() && IMU::isHealthy() && IMU::isStationary() && IMU::isUpright() && Latch::isClosed();
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    char cmd = (char) Serial.read();
    if (cmd == 's') emergencyStop();
    if (cmd == 't') { Simulation::stepHeld = true; Serial.println("MOCK: step stuck"); }
    if (cmd == 'n') { Simulation::stepHeld = false; Serial.println("MOCK: step unstuck"); }
    if (cmd == 'b') { Battery::setHealthy(false); Serial.println("MOCK: low battery"); }
    if (cmd == 'g') { Battery::setHealthy(true); Serial.println("MOCK: battery OK"); }
    if (cmd == 'i') { IMU::setHealthy(false); Serial.println("MOCK: IMU failed"); }
    if (cmd == 'k') { IMU::setHealthy(true); Serial.println("MOCK: IMU OK"); }
    if (cmd == 'u') { IMU::setUpright(false); Serial.println("MOCK: not upright"); }
    if (cmd == 'o') { IMU::setUpright(true); Serial.println("MOCK: upright"); }

    if (cmd == 'r') {
      if (currentState != EMERGENCY_STOP && currentState != FAULT) {
        Serial.println("Restart allowed only from STOP or FAULT");
      } 
      else if (digitalRead(STOP_BUTTON_PIN) == LOW || !Battery::isHealthy() || !IMU::isHealthy()) {
        Serial.println("Cannot restart: button pressed or fault remains");
      } 
      else {
        safeStopAll();
        IMU::resetPose();
        Latch::begin();
        Simulation::stepHeld = false;
        Serial.println("MANUAL RESTART (simulation)");
        changeState(SETTLE);
      }
    }
  }
}

// Commands run once on entry; loop() only polls completion.
void startStateOperation(State state) {
  switch (state) {
    case SETTLE: IMU::settle(); break;
    case WIND: Winch::wind(); break;
    case LATCH: Latch::close(); break;
    case BACK_OFF: Winch::backOff(); break;
    case FOLD_LEGS: Legs::fold(); break;
    case READY: IMU::checkReady(); break;
    case RELEASE:
      if (!safeToRelease()) enterFault("release interlock failed");
      else Latch::release();
      break;
    case FLYING: IMU::fly(); break;
    case LANDED: IMU::land(); break;
    case STAND_UP: Legs::stand(); break;
    case GET_READY_AGAIN: Latch::reset(); break;
    case EMERGENCY_STOP:
    case FAULT: break;
  }
}

void pollState() {
  switch (currentState) {
    case SETTLE: if (IMU::isComplete()) changeState(WIND); break;
    case WIND: if (Winch::isComplete()) changeState(LATCH); break;
    case LATCH: if (Latch::isComplete() && Latch::isClosed()) changeState(BACK_OFF); break;
    case BACK_OFF: if (Winch::isComplete()) changeState(FOLD_LEGS); break;
    case FOLD_LEGS: if (Legs::isComplete()) changeState(READY); break;
    case READY:
      if (IMU::isComplete()) {
        if (!safeToRelease()) enterFault("release interlock failed");
        else changeState(RELEASE);
      }
      break;
    case RELEASE:
      if (Latch::isComplete()) {
        Serial.println("MOCK ONLY: release permitted; no physical output");
        changeState(FLYING);
      }
      break;
    case FLYING: if (IMU::isComplete()) changeState(LANDED); break;
    case LANDED: if (IMU::isComplete() && IMU::isStationary()) changeState(STAND_UP); break;
    case STAND_UP: if (Legs::isComplete() && IMU::isUpright()) changeState(GET_READY_AGAIN); break;
    case GET_READY_AGAIN: if (Latch::isComplete()) changeState(SETTLE); break;
    case EMERGENCY_STOP:
    case FAULT: break;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);
  Legs::begin();
  Winch::begin();
  Latch::begin();
  bool imuStarted = IMU::begin();
  bool batteryStarted = Battery::begin();

  Serial.println("D6 SIMULATION STARTED (NO HARDWARE OUTPUT)");
  if (!imuStarted || !batteryStarted) {
    enterFault("module failed to start");
  }
  else {
    changeState(SETTLE);
  }
}

void loop() {
  // Check stop first, EVERY iteration, regardless of the current step.
  if (digitalRead(STOP_BUTTON_PIN) == LOW) emergencyStop();
  readSerialCommands();

  // Fault/stop are latched until a deliberate 'r' command.
  if (currentState == EMERGENCY_STOP || currentState == FAULT) {
    safeStopAll();
    return;
  }

  if (!Battery::isHealthy()) { enterFault("battery failure"); return; }
  if (!IMU::isHealthy())     { enterFault("IMU failure"); return; }

  // Recheck before updating the mock release, so an interlock failure cancels it.
  if (currentState == RELEASE && !safeToRelease()) {
    enterFault("release interlock failed");
    return;
  }

  // Timeout takes priority over completion, including after a long loop pause.
  if (millis() - stateStartedAt >= STEP_TIMEOUT_MS) {
    enterFault("step timed out");
    return;
  }
  Legs::update();
  Winch::update();
  Latch::update();
  IMU::update();
  Battery::update();

  /* Check again in case we timed out during the updates */
  if (millis() - stateStartedAt >= STEP_TIMEOUT_MS) {
    enterFault("step timed out");
    return;
  }

  pollState();
}

