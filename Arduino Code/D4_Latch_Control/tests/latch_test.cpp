#include <cassert>
#include <cstdint>
#include <iostream>
#include "Arduino.h"
uint32_t clockMs = 0;
unsigned long millis() { return clockMs; }
MockSerial Serial;
#include "../D4_Latch_Control.ino"

struct SafetyConditions {
  bool stationary = false, upright = false, batteryHealthy = false, imuHealthy = false;
  bool emergencyStop = false, faultActive = false;
};
void supply(SafetyConditions s) {
  Latch::setSafetyConditions(s.stationary, s.upright, s.batteryHealthy,
                             s.imuHealthy, s.emergencyStop, s.faultActive);
}
SafetyConditions healthy() {
  SafetyConditions s;
  s.stationary = s.upright = s.batteryHealthy = s.imuHealthy = true;
  return s;
}
void tick(uint32_t ms) { clockMs += ms; Latch::update(); }
void fresh(uint32_t start = 0) {
  // Reset through the public operator interface, never via internal storage.
  Latch::stop();
  Latch::simulateStuck(false);
  supply(healthy());
  assert(Latch::resetLockout());
  clockMs = start;
  Latch::begin();
  assert(Latch::status() == Latch::IDLE && !Latch::isComplete());
}
void closed() {
  assert(Latch::close());
  assert(!Latch::isComplete() && !Latch::isClosed());
  tick(799); assert(Latch::status() == Latch::CLOSING);
  assert(!Latch::close()); // Repeated commands cannot restart the timer.
  tick(1); assert(Latch::status() == Latch::CLOSED);
  assert(Latch::isClosed() && Latch::isComplete());
}
void failInput(SafetyConditions& s, int which) {
  switch (which) {
    case 0: s.stationary = false; break;
    case 1: s.upright = false; break;
    case 2: s.batteryHealthy = false; break;
    case 3: s.imuHealthy = false; break;
    case 4: s.emergencyStop = true; break;
    case 5: s.faultActive = true; break;
  }
}
void assertLocked() {
  assert(Latch::status() == Latch::FAULT && !Latch::isComplete());
  assert(!Latch::requestRelease() && !Latch::close());
  Latch::begin();
  tick(5000);
  assert(Latch::status() == Latch::FAULT);
}
void testNormal() {
  fresh(); assert(!Latch::requestRelease()); closed();
  assert(Latch::requestRelease());
  assert(!Latch::isComplete() && Latch::isClosed());
  assert(!Latch::requestRelease() && !Latch::close());
  tick(799); assert(Latch::status() == Latch::RELEASING);
  tick(1); assert(Latch::status() == Latch::RELEASED);
  assert(!Latch::isClosed() && Latch::isComplete());
  assert(!Latch::requestRelease()); closed();
}
void testInterlocks() {
  for (int i = 0; i < 6; ++i) {
    fresh(); closed(); auto s = healthy(); failInput(s, i);
    supply(s);
    assert(!Latch::requestRelease());
    tick(800); assert(Latch::isClosed());
    // Pending failure must cancel even at the completion boundary.
    fresh(); closed(); assert(Latch::requestRelease());
    clockMs += 800;
    supply(s);
    supply(healthy()); // Restoration cannot erase a failure.
    assertLocked();
    assert(Latch::isClosed());
    assert(Latch::resetLockout());
    assert(!Latch::requestRelease());
    // Every unsafe input must also prevent reset.
    Latch::stop(); supply(s);
    assert(!Latch::resetLockout());
  }
  fresh(); closed(); Latch::simulateLostClosure(); assertLocked();
  fresh(); closed(); assert(Latch::requestRelease());
  Latch::simulateLostClosure(); assertLocked();
  fresh(); supply(SafetyConditions()); closed();
  assert(!Latch::requestRelease()); // Unsupplied readings fail closed.
}
void testPollingInterlocks() {
  const char* reasons[] = {"hopper moving", "hopper not upright", "battery unhealthy",
                           "IMU unhealthy", "emergency stop", "external fault active"};
  for (int i = 0; i < 6; ++i) {
    fresh(); closed();
    auto unsafe = healthy(); failInput(unsafe, i); supply(unsafe);
    assert(!Latch::requestRelease());
    assert(std::string(Latch::message) == reasons[i]);

    // Exercise update independently of the input setter at completion time.
    fresh(); closed(); assert(Latch::requestRelease());
    if (i == 0) Latch::stationary = false;
    else if (i == 1) Latch::upright = false;
    else if (i == 2) Latch::batteryHealthy = false;
    else if (i == 3) Latch::imuHealthy = false;
    else if (i == 4) Latch::emergencyStop = true;
    else Latch::faultActive = true;
    tick(800);
    assertLocked();
    assert(Latch::isClosed());
    assert(std::string(Latch::faultReason) == reasons[i]);
  }
  // An external fault must also cancel closing and latch while idle or closed.
  for (int phase = 0; phase < 3; ++phase) {
    fresh();
    if (phase == 1) assert(Latch::close());
    if (phase == 2) closed();
    auto unsafe = healthy(); unsafe.faultActive = true; supply(unsafe);
    assertLocked(); supply(healthy()); assertLocked();
    assert(Latch::resetLockout());
  }
}
void testStops() {
  for (int phase = 0; phase < 4; ++phase) {
    fresh();
    if (phase == 1) assert(Latch::close());
    if (phase >= 2) closed();
    if (phase == 3) assert(Latch::requestRelease());
    Latch::inhibitRelease(); assertLocked();
    supply(healthy()); assertLocked();
    assert(Latch::resetLockout()); closed();
  }
  fresh(); assert(Latch::close()); Latch::stop(); assertLocked();
}
void testTimeoutsAndRollover() {
  for (int release = 0; release < 2; ++release) {
    for (int wrap = 0; wrap < 2; ++wrap) {
      fresh(wrap ? UINT32_MAX - 400 : 0);
      if (release) {
        closed();
        if (wrap) clockMs = UINT32_MAX - 400;
        assert(Latch::requestRelease());
      }
      else assert(Latch::close());
      Latch::simulateStuck(true);
      tick(2999); assert(Latch::status() != Latch::FAULT);
      tick(1); assert(Latch::fault() == Latch::OPERATION_TIMEOUT);
      assertLocked(); assert(!Latch::resetLockout());
      Latch::simulateStuck(false); assert(Latch::resetLockout());
    }
    fresh();
    if (release) { closed(); assert(Latch::requestRelease()); }
    else assert(Latch::close());
    tick(3000); // Long polling gap: timeout beats overdue completion.
    assert(Latch::fault() == Latch::OPERATION_TIMEOUT);
  }
  fresh(UINT32_MAX - 400); closed();
  assert(Latch::requestRelease()); tick(800);
  assert(Latch::status() == Latch::RELEASED);
}
void testFirstFaultRetained() {
  fresh(); closed(); assert(Latch::requestRelease());
  auto unsafe = healthy();
  unsafe.batteryHealthy = false;
  supply(unsafe);
  assert(Latch::fault() == Latch::INTERLOCK_FAILED);
  Latch::stop();
  unsafe.emergencyStop = true;
  supply(unsafe);
  tick(3000);
  assert(Latch::fault() == Latch::INTERLOCK_FAILED);
  assertLocked();
  supply(healthy());
  assert(Latch::resetLockout());
  assert(Latch::fault() == Latch::NONE);
  assert(!Latch::isClosed() && !Latch::requestRelease());

  // A later external fault must also preserve an earlier emergency stop.
  Latch::stop();
  unsafe = healthy();
  unsafe.faultActive = true;
  supply(unsafe);
  assert(Latch::fault() == Latch::EMERGENCY_STOP);
  assertLocked();
}
void command(const char* text) { Serial.input = text; loop(); }
void testSketch() {
  fresh(); setup(); command("c"); clockMs += 800; loop();
  assert(Latch::isClosed()); command("j");
  command("s"); assertLocked();
  command("r"); assert(Latch::status() == Latch::IDLE);
  command("c"); clockMs += 800; loop(); command("j");
  clockMs += 800; loop(); assert(Latch::status() == Latch::RELEASED);
  command("s");
  auto unsafe = healthy(); unsafe.emergencyStop = true; supply(unsafe);
  command("r"); assert(Latch::status() == Latch::FAULT);
  supply(healthy()); command("r"); assert(Latch::status() == Latch::IDLE);
  Serial.output.clear(); command("j");
  assert(Serial.output.find("Request refused\nlatch not confirmed closed") != std::string::npos);
  command("c"); clockMs += 800; loop(); command("j");
  unsafe = healthy(); unsafe.imuHealthy = false; supply(unsafe); loop();
  assert(Serial.output.find("IMU unhealthy") != std::string::npos);
  supply(healthy()); command("r");
  for (char removed : std::string("?mvuobgikefatnlx")) {
    Serial.output.clear();
    char input[] = {removed, 0}; command(input);
    assert(Latch::status() == Latch::IDLE);
    assert(Serial.output == "Unknown command\n");
  }

}
int main() {
  // Actual initial state has no valid readings.
  Latch::begin(); assert(!Latch::requestRelease());
  testNormal(); testInterlocks(); testPollingInterlocks(); testStops(); testTimeoutsAndRollover(); testFirstFaultRetained(); testSketch();
  std::cout << "PASS: closure, release, all interlocks, pending cancellation, validated reset,\n"
               "stop lockout, first-fault retention, timeouts, rollover, repeated cycles, and Serial sketch.\n";
}
