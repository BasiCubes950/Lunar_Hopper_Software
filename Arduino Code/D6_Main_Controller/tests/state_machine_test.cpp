#include <cassert>
#include <climits>
#include <iostream>
#include <string>

// Minimal Arduino simulation: no real clock, pins, or hardware needed.
using byte = unsigned char;
const int LOW = 0;
const int HIGH = 1;
const int INPUT_PULLUP = 2;
unsigned long clockMs = 0;
int stopButton = HIGH;
unsigned long millis() { return clockMs; }
int digitalRead(byte) { return stopButton; }
void pinMode(byte, int mode) { assert(mode == INPUT_PULLUP); }
struct MockSerial {
  std::string input;
  std::string output;
  void begin(int baud) { assert(baud == 115200); }
  int available() { return static_cast<int>(input.size()); }
  char read() { char c = input.front(); input.erase(0, 1); return c; }
  void print(const char* value) { output += value; }
  void println(const char* value) { output += value; output += '\n'; }
} Serial;

#include "../D6_Main_Controller.ino"

void boot(unsigned long start = 0) {
  clockMs = start;
  stopButton = HIGH;
  Serial.input.clear();
  Serial.output.clear();
  Simulation::stepHeld = false;
  currentState = SETTLE;
  setup();
  assert(currentState == SETTLE);
}
void tick(unsigned long elapsed) { clockMs += elapsed; loop(); }
void command(char c) { Serial.input += c; loop(); }
void reach(State target) {
  for (int count = 0; currentState != target && count < 11; count++) tick(STEP_DURATION_MS);
  assert(currentState == target);
}
void assertStopped() {
  assert(!Legs::operation.active);
  assert(!Winch::operation.active);
  assert(!Latch::operation.active);
  assert(!IMU::operation.active);
  assert(Latch::releaseInhibited);
}

void testProgression() {
  boot();
  // All eleven operations must take their full duration, even with frequent polls.
  for (int state = SETTLE; state <= GET_READY_AGAIN; state++) {
    assert(currentState == static_cast<State>(state));
    for (unsigned long elapsed = 0; elapsed < STEP_DURATION_MS - 1; elapsed++) tick(1);
    assert(currentState == static_cast<State>(state));
    tick(1);
    State expected = state == GET_READY_AGAIN ? SETTLE : static_cast<State>(state + 1);
    assert(currentState == expected);
    if (currentState == BACK_OFF) assert(Latch::isClosed());
    if (currentState == FLYING) assert(!Latch::isClosed() && !IMU::isStationary());
    if (currentState == STAND_UP) { assert(IMU::isStationary()); IMU::setUpright(false); }
    if (currentState == GET_READY_AGAIN) assert(IMU::isUpright());
  }
  reach(RELEASE); // A second cycle must also work.
}
void testStopsAndTimeouts() {
  for (int state = SETTLE; state <= GET_READY_AGAIN; state++) {
    for (int physical = 0; physical < 2; physical++) {
      boot(); reach(static_cast<State>(state));
      if (physical) { stopButton = LOW; loop(); }
      else command('s');
      assert(currentState == EMERGENCY_STOP);
      assertStopped();
      tick(STEP_TIMEOUT_MS + 1);
      assert(currentState == EMERGENCY_STOP);
      if (physical) { command('r'); assert(currentState == EMERGENCY_STOP); }
      stopButton = HIGH;
      command('r');
      assert(currentState == SETTLE);
      assert(IMU::isStationary() && IMU::isUpright() && !Latch::isClosed());
    }
    boot(); reach(static_cast<State>(state)); command('t');
    tick(STEP_TIMEOUT_MS - 1);
    assert(currentState == static_cast<State>(state));
    tick(1);
    assert(currentState == FAULT);
    assertStopped();
    assert(Serial.output.find("step timed out") != std::string::npos);
    command('n'); tick(STEP_DURATION_MS);
    assert(currentState == FAULT);
    command('r'); assert(currentState == SETTLE);
  }
  boot(); command('t'); tick(400); command('n'); tick(400);
  assert(currentState == WIND);
  boot(); tick(STEP_TIMEOUT_MS); assert(currentState == FAULT);
}
void failInterlock(int failure) {
  if (failure == 0) Battery::setHealthy(false);
  else if (failure == 1) IMU::setHealthy(false);
  else if (failure == 2) IMU::setStationary(false);
  else if (failure == 3) IMU::setUpright(false);
  else Latch::begin(); // Simulate a lost latch-closed confirmation.
}
void testReleaseFailures() {
  for (int state = READY; state <= RELEASE; state++) {
    for (int failure = 0; failure < 5; failure++) {
      boot(); reach(static_cast<State>(state));
      failInterlock(failure);
      tick(STEP_DURATION_MS);
      assert(currentState == FAULT);
      assertStopped();
      assert(Serial.output.find("release permitted") == std::string::npos);
    }
  }
  // Losing an interlock partway through release cancels it immediately.
  boot(); reach(RELEASE); tick(400); command('u');
  assert(currentState == FAULT); assert(Latch::isClosed());
  command('o'); tick(STEP_DURATION_MS); assert(currentState == FAULT);
  command('r'); assert(currentState == SETTLE);
}
void testSensorCommandsAndRestart() {
  for (int sensor = 0; sensor < 2; sensor++) {
    boot(); command(sensor == 0 ? 'b' : 'i');
    assert(currentState == FAULT); assertStopped();
    command('r'); assert(currentState == FAULT);
    command(sensor == 0 ? 'g' : 'k');
    assert(currentState == FAULT);
    command('r'); assert(currentState == SETTLE);
  }
  boot(); reach(WIND); command('r'); assert(currentState == WIND);
  command('u'); assert(!IMU::isUpright()); command('o'); assert(IMU::isUpright());
}
void testClockRollover() {
  boot(ULONG_MAX - 400);
  tick(STEP_DURATION_MS - 1); assert(currentState == SETTLE);
  tick(1); assert(currentState == WIND);
  boot(ULONG_MAX - 400); command('t');
  tick(STEP_TIMEOUT_MS); assert(currentState == FAULT);
}
int main() {
  testProgression();
  testStopsAndTimeouts();
  testReleaseFailures();
  testSensorCommandsAndRestart();
  testClockRollover();
  std::cout << "All D6 state machine tests passed.\n";
}
