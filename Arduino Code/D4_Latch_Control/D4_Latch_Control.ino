// SIMULATION ONLY: no pins or physical latch outputs.
#include "LatchModule.h"

Latch::Status reportedStatus = Latch::IDLE;

void printStatus() {
  reportedStatus = Latch::status();
  Serial.print("D4: "); Serial.println(Latch::statusName());
  if (Latch::status() == Latch::FAULT) Serial.println(Latch::faultReason);
}

void setup() {
  Serial.begin(115200);
  Latch::begin();
  Latch::setSafetyConditions(true, true, true, true);
  Serial.println("D4 SIMULATION: c close, j release, s stop, r validated reset");
  printStatus();
}

void loop() {
  while (Serial.available() > 0) {
    char cmd = static_cast<char>(Serial.read());
    if (cmd == '\r' || cmd == '\n' || cmd == ' ') continue;
    bool accepted = false;
    if (cmd == 'c') accepted = Latch::close();
    else if (cmd == 'j') accepted = Latch::requestRelease();
    else if (cmd == 's') { Latch::stop(); accepted = true; }
    else if (cmd == 'r') accepted = Latch::resetLockout();
    else { Serial.println("Unknown command"); continue; }
    Serial.println(accepted ? "Request accepted" : "Request refused");
    if (!accepted) Serial.println(Latch::message);
    printStatus();
  }
  Latch::update();
  if (Latch::status() != reportedStatus) printStatus();
}
