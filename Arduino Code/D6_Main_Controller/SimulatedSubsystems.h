#ifndef SIMULATED_SUBSYSTEMS_H
#define SIMULATED_SUBSYSTEMS_H

// All operations below change memory only. There are no actuator output pins.
namespace Simulation {
  bool stepHeld = false;

  // A reusable non-blocking timer. Each subsystem owns its own operation.
  struct Operation {
    unsigned long startedAt = 0;
    bool active = false;
    bool complete = false;

    void start() {
      startedAt = millis();
      active = true;
      complete = false;
    }
    void update() {
      if (active && !stepHeld && millis() - startedAt >= STEP_DURATION_MS) {
        active = false;
        complete = true;
      }
    }
    void stop() {
      active = false;
      complete = false;
    }
  };
}

namespace Battery {
  bool healthy = true;
  bool begin() { healthy = true; return true; }
  void update() {} // Serial commands provide this simulated sensor reading.
  bool isHealthy() { return healthy; }
  void setHealthy(bool value) { healthy = value; }
}

namespace IMU {
  enum Action { SETTLING, CHECKING_READY, FLIGHT, LANDING };
  Action action = SETTLING;
  Simulation::Operation operation;
  bool healthy = true;
  bool stationary = true;
  bool upright = true;

  void resetPose() { stationary = true; upright = true; operation.stop(); }
  bool begin() { healthy = true; resetPose(); return true; }
  void settle() { action = SETTLING; operation.start(); }
  void checkReady() { action = CHECKING_READY; operation.start(); }
  void fly() { action = FLIGHT; stationary = false; operation.start(); }
  void land() { action = LANDING; operation.start(); }
  void update() {
    bool wasActive = operation.active;
    operation.update();
    if (wasActive && operation.complete && (action == SETTLING || action == LANDING)) stationary = true;
  }
  void stop() { operation.stop(); }
  bool isComplete() { return operation.complete; }
  bool isHealthy() { return healthy; }
  bool isStationary() { return stationary; }
  bool isUpright() { return upright; }
  void setHealthy(bool value) { healthy = value; }
  void setStationary(bool value) { stationary = value; }
  void setUpright(bool value) { upright = value; }
}

namespace Legs {
  enum Action { FOLDING, STANDING };
  Action action = FOLDING;
  Simulation::Operation operation;
  void begin() { operation.stop(); }
  void fold() { action = FOLDING; operation.start(); }
  void stand() { action = STANDING; operation.start(); }
  void update() {
    bool wasActive = operation.active;
    operation.update();
    // Simulate the pose sensor confirming the completed stand operation.
    if (wasActive && operation.complete && action == STANDING) IMU::setUpright(true);
  }
  void stop() { operation.stop(); }
  bool isComplete() { return operation.complete; }
}

namespace Winch {
  Simulation::Operation operation;
  void begin() { operation.stop(); }
  void wind() { operation.start(); }
  void backOff() { operation.start(); }
  void update() { operation.update(); }
  void stop() { operation.stop(); }
  bool isComplete() { return operation.complete; }
}

namespace Latch {
  enum Action { CLOSING, RELEASING, RESETTING };
  Action action = RESETTING;
  Simulation::Operation operation;
  bool closed = false;
  bool releaseInhibited = true;
  void begin() { operation.stop(); closed = false; releaseInhibited = true; }
  void close() { action = CLOSING; releaseInhibited = true; operation.start(); }
  void release() { action = RELEASING; releaseInhibited = false; operation.start(); }
  void reset() { action = RESETTING; releaseInhibited = true; operation.start(); }
  void update() {
    bool wasActive = operation.active;
    operation.update();
    if (wasActive && operation.complete) {
      if (action == CLOSING) closed = true;
      else if (action == RESETTING || (action == RELEASING && !releaseInhibited)) closed = false;
    }
  }
  void inhibitRelease() { releaseInhibited = true; operation.stop(); }
  bool isClosed() { return closed; }
  bool isComplete() { return operation.complete; }
}

#endif
