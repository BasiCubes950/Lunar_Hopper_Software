#ifndef STAC_D4_LATCH_MODULE_H
#define STAC_D4_LATCH_MODULE_H

#include <Arduino.h>
#include <stdint.h>

// SIMULATION ONLY. This module never writes pins or controls a physical latch.
namespace Latch {
  enum Status { IDLE, CLOSING, CLOSED, RELEASING, RELEASED, FAULT };
  enum Fault { NONE, EMERGENCY_STOP, INTERLOCK_FAILED, OPERATION_TIMEOUT };

  // Missing sensor readings must prevent release and reset.
  bool stationary = false;
  bool upright = false;
  bool batteryHealthy = false;
  bool imuHealthy = false;
  bool emergencyStop = false;
  bool faultActive = false;
  const uint32_t OPERATION_DURATION_MS = 800;
  const uint32_t OPERATION_TIMEOUT_MS = 3000;

  // State belongs directly to Latch, like the D6 simulated subsystems.
  // Include this module in one translation unit (the Arduino sketch).
  Status currentStatus = IDLE;
  Fault currentFault = NONE;
  const char* faultReason = "none";
  const char* message = "ready";
  bool closureConfirmed = false;
  bool operationComplete = false;
  bool operationStuck = false;
  uint32_t operationStartedAt = 0;

  // Return the first unsafe input for a readable failure message.
  const char* unsafeReason() {
    if (emergencyStop) return "emergency stop input active";
    if (faultActive) return "external fault active";
    if (!stationary) return "hopper moving";
    if (!upright) return "hopper not upright";
    if (!batteryHealthy) return "battery unhealthy";
    if (!imuHealthy) return "IMU unhealthy";
    return nullptr;
  }

  bool safetyConditionsMet() { return unsafeReason() == nullptr; }

  // Cancel the operation and keep the first fault until an explicit reset.
  void enterFault(Fault reason, const char* detail) {
    currentStatus = FAULT;
    operationComplete = false;
    if (currentFault == NONE) {
      currentFault = reason;
      faultReason = detail;
    }
  }

  Status status() { return currentStatus; }

  Fault fault() { return currentFault; }

  bool isClosed() { return closureConfirmed; }

  bool isComplete() { return operationComplete; }

  // Initial values are set above; repeated calls must never clear a lockout.
  void begin() {}

  // Cancel any operation and require an explicit validated reset.
  void inhibitRelease() { enterFault(EMERGENCY_STOP, "emergency stop"); }

  void stop() { inhibitRelease(); }

  // Recheck safety and advance the simulated operation without blocking.
  void update() {
    if (emergencyStop) {
      inhibitRelease();
      return;
    }
    if (faultActive) {
      enterFault(INTERLOCK_FAILED, unsafeReason());
      return;
    }
    if (currentStatus == FAULT) {
      return;
    }
    // Interlocks take priority, even on the update that would finish release.
    if (currentStatus == RELEASING &&
        (!closureConfirmed || !safetyConditionsMet())) {
      enterFault(INTERLOCK_FAILED, !closureConfirmed ? "closure lost" : unsafeReason());
      return;
    }
    if (currentStatus != CLOSING && currentStatus != RELEASING) {
      return;
    }

    // Unsigned 32-bit subtraction handles millis() rollover in host tests too.
    uint32_t elapsed = static_cast<uint32_t>(millis()) - operationStartedAt;
    // Timeout takes priority over completion after a long polling gap.
    if (elapsed >= OPERATION_TIMEOUT_MS) {
      enterFault(OPERATION_TIMEOUT, "operation timed out");
      return;
    }
    if (!operationStuck && elapsed >= OPERATION_DURATION_MS) {
      if (currentStatus == CLOSING) {
        closureConfirmed = true;
        currentStatus = CLOSED;
      }
      else {
        closureConfirmed = false;
        currentStatus = RELEASED;
      }
      operationComplete = true;
    }
  }

  // Save readings and check them immediately, before they can be overwritten.
  void setSafetyConditions(bool still, bool vertical, bool batteryOK, bool imuOK,
                           bool stopActive = false, bool externalFault = false) {
    stationary = still;
    upright = vertical;
    batteryHealthy = batteryOK;
    imuHealthy = imuOK;
    emergencyStop = stopActive;
    faultActive = externalFault;
    update();
  }

  // Start simulated closure unless an operation or lockout is already active.
  bool close() {
    update();
    if (currentStatus == FAULT || currentStatus == CLOSING || currentStatus == RELEASING) {
      message = currentStatus == FAULT ? faultReason : "operation busy";
      return false;
    }
    if (currentStatus == CLOSED && closureConfirmed) {
      return true;
    }
    currentStatus = CLOSING;
    operationComplete = false;
    operationStartedAt = static_cast<uint32_t>(millis());
    return true;
  }

  // Start release only after confirmed closure and with every interlock safe.
  bool requestRelease() {
    update();
    if (currentStatus == FAULT) message = faultReason;
    else if (currentStatus != CLOSED || !closureConfirmed) message = "latch not confirmed closed";
    else if (!safetyConditionsMet()) message = unsafeReason();
    else {
      currentStatus = RELEASING;
      operationComplete = false;
      operationStartedAt = static_cast<uint32_t>(millis());
      return true;
    }
    return false;
  }

  // Acknowledge a fault only when safe; require a fresh close cycle afterwards.
  bool resetLockout() {
    if (currentStatus != FAULT) message = "no fault or stop to reset";
    else if (!safetyConditionsMet()) message = unsafeReason();
    else if (operationStuck) message = "operation still stuck";
    else {
      currentStatus = IDLE;
      currentFault = NONE;
      faultReason = "none";
      closureConfirmed = false;
      operationComplete = false;
      return true;
    }
    return false;
  }

  // Hold simulated completion to exercise the operation timeout.
  void simulateStuck(bool stuck) { operationStuck = stuck; }

  // Simulate lost closure feedback and cancel a closed or releasing state.
  void simulateLostClosure() {
    closureConfirmed = false;
    if (currentStatus == CLOSED || currentStatus == RELEASING) {
      enterFault(INTERLOCK_FAILED, "closure lost");
    }
  }

  const char* statusName() {
    switch (status()) {
      case IDLE: return "IDLE";
      case CLOSING: return "CLOSING";
      case CLOSED: return "CLOSED";
      case RELEASING: return "RELEASING";
      case RELEASED: return "RELEASED";
      case FAULT: return "FAULT";
    }
    return "UNKNOWN";
  }
}


#endif
