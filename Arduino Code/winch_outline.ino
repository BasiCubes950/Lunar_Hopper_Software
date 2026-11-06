/*
  Hopper winch: code outline (stub version)
  -----------------------------------------
  The winch is a motor + spool. It pulls a cable that bends carbon-fiber rods
  (the jump spring). This sketch:
    1. winds the cable to a chosen length (measured in encoder counts)
    2. stops safely on a stall (high current but no movement) or a timeout
    3. after the latch closes, lets out a little cable so the LATCH holds the
       load and not the motor

  SIMULATE = true  -> no hardware needed; fake encoder/current/latch values.
  SIMULATE = false -> reads the real encoder, current sensor and latch switch
                      (to be tested and calibrated in person).

  Serial Monitor commands (9600 baud, "Newline" line ending):
    w = start winding      x = emergency stop     r = reset after fault
    s = toggle simulated stall (motor stuck)
    k = toggle simulated slow winding (to trigger the timeout)
    l = toggle simulated latch closed
*/

#define SIMULATE true

// ---------------------------------------------------------------------------
// Pins (Nano). AGREE THESE WITH THE D1 PERSON before wiring.
// The Servo library disables analogWrite (PWM) on pins 9 and 10, so the motor
// speed pin is NOT 9 or 10. Pin 5 is a PWM pin that Servo does not touch.
// Encoder A must be on an interrupt pin (Nano: 2 or 3).
// ---------------------------------------------------------------------------
const uint8_t PIN_MOTOR_PWM = 5;   // motor driver speed input (PWM)
const uint8_t PIN_MOTOR_DIR = 4;   // motor driver direction input
const uint8_t PIN_ENC_A     = 2;   // encoder channel A (interrupt)
const uint8_t PIN_ENC_B     = 3;   // encoder channel B (direction)
const uint8_t PIN_CURRENT   = A0;  // motor current sense (analog)
const uint8_t PIN_LATCH     = 7;   // latch closed switch (LOW = closed)

// ---------------------------------------------------------------------------
// Tunable settings (placeholder numbers: calibrate with the real motor)
// ---------------------------------------------------------------------------
const long     WIND_TARGET_COUNTS   = 2000;  // how far to wind (encoder counts)
const long     SLACK_COUNTS         = 60;    // how much cable to let out after latching
const uint8_t  WIND_PWM             = 200;   // winding speed (0-255)
const uint8_t  SLACK_PWM            = 90;    // slow speed for letting out slack
const long     TARGET_TOLERANCE     = 5;     // "close enough" to target (counts)

// Stall rule: current high AND cable not moving, for several checks in a row
const int      STALL_CURRENT_LIMIT  = 600;   // ADC reading (0-1023) counted as "working hard"
const long     STALL_MIN_COUNTS     = 3;     // less movement than this per window = "not moving"
const unsigned long STALL_WINDOW_MS = 150;   // how often we check movement
const uint8_t  STALL_WINDOWS_TO_TRIP = 3;    // consecutive bad windows before stopping
const unsigned long STARTUP_GRACE_MS = 300;  // ignore stall check while motor spins up (inrush current)

// Timeout rule: stop if winding takes much longer than expected
const long     EXPECTED_COUNTS_PER_SEC = 1000;  // expected winding speed (calibrate)
const float    TIMEOUT_FACTOR       = 1.5;      // allow 50% longer than expected
const unsigned long TIMEOUT_MARGIN_MS = 500;    // extra fixed allowance

// Latch and slack steps
const unsigned long LATCH_WAIT_MS   = 5000;  // max time to wait for the latch to close
const unsigned long SLACK_TIMEOUT_MS = 2000; // max time to let out slack

// ---------------------------------------------------------------------------
// State machine
// ---------------------------------------------------------------------------
enum WinchState { IDLE, WINDING, WAIT_LATCH, LETTING_OUT, HOLDING, FAULT };
WinchState state = IDLE;

const char* stateName(WinchState s) {
  switch (s) {
    case IDLE:        return "IDLE";
    case WINDING:     return "WINDING";
    case WAIT_LATCH:  return "WAIT_LATCH";
    case LETTING_OUT: return "LETTING_OUT";
    case HOLDING:     return "HOLDING";
    default:          return "FAULT";
  }
}

// Winding bookkeeping
long          targetCounts   = 0;
unsigned long stateStartMs   = 0;   // when the current state began
unsigned long windTimeoutMs  = 0;   // computed time limit for this wind
unsigned long lastStallCheckMs = 0;
long          lastStallCheckPos = 0;
uint8_t       badWindows     = 0;
long          slackStartPos  = 0;
const char*   faultReason    = "";
uint8_t       motorPwmNow    = 0;   // what we last told the motor
bool          motorWindDir   = true;

// Encoder (real hardware): updated in an interrupt
volatile long encoderCount = 0;

// Simulation state
bool  simStall      = false;
bool  simSlow       = false;
bool  simLatch      = false;
float simPos        = 0;
unsigned long simLastMs = 0;

// ---------------------------------------------------------------------------
// Low-level hardware access (real or simulated)
// ---------------------------------------------------------------------------

/* Encoder interrupt: counts +1 or -1 depending on direction (channel B). */
void encoderISR() {
  if (digitalRead(PIN_ENC_B) == HIGH) encoderCount++;
  else                                encoderCount--;
}

/* Fake physics for SIMULATE mode. Moves the pretend cable while the motor
   runs, unless a stall or slow-wind is being simulated. */
void updateSimulation() {
  unsigned long now = millis();
  float dt = (now - simLastMs) / 1000.0;
  simLastMs = now;
  if (motorPwmNow == 0 || simStall) return;     // stuck motor: no movement
  float speed = (float)motorPwmNow / 255.0 * 1200.0;   // counts per second
  if (simSlow) speed *= 0.25;                   // pretend it is dragging
  simPos += (motorWindDir ? 1 : -1) * speed * dt;
}

/* Cable position in encoder counts (0 = fully unwound). */
long readPosition() {
#if SIMULATE
  return (long)simPos;
#else
  noInterrupts();               // copy safely: the ISR can change it anytime
  long p = encoderCount;
  interrupts();
  return p;
#endif
}

/* Motor current as a raw ADC reading (0-1023). Higher = working harder. */
int readCurrent() {
#if SIMULATE
  if (motorPwmNow == 0) return 0;
  if (simStall) return 900;                     // stuck: current shoots up
  return 150 + motorPwmNow;                     // normal load
#else
  return analogRead(PIN_CURRENT);
#endif
}

/* True when the latch is closed and can hold the load. */
bool latchClosed() {
#if SIMULATE
  return simLatch;
#else
  return digitalRead(PIN_LATCH) == LOW;         // switch to ground when closed
#endif
}

// ---------------------------------------------------------------------------
// Motor control
// ---------------------------------------------------------------------------

/* Set motor speed (0-255) and direction (true = wind in, false = let out). */
void motorDrive(uint8_t pwm, bool windIn) {
  motorPwmNow  = pwm;
  motorWindDir = windIn;
#if !SIMULATE
  digitalWrite(PIN_MOTOR_DIR, windIn ? HIGH : LOW);
  analogWrite(PIN_MOTOR_PWM, pwm);
#endif
}

/* Stop the motor immediately. Safe to call at any time. */
void stopMotor() {
  motorDrive(0, motorWindDir);
}

/* Begin winding to a target length. Ignored unless the winch is IDLE. */
void startWinding(long target) {
  if (state != IDLE) {
    Serial.println(F("Cannot start: winch is not IDLE (send r after a fault)."));
    return;
  }
  targetCounts      = target;
  long remaining    = target - readPosition();
  if (remaining <= TARGET_TOLERANCE) {
    Serial.println(F("Already at or past target."));
    return;
  }
  // Time limit = expected time * factor + margin
  float expectedMs  = (float)remaining / EXPECTED_COUNTS_PER_SEC * 1000.0;
  windTimeoutMs     = (unsigned long)(expectedMs * TIMEOUT_FACTOR) + TIMEOUT_MARGIN_MS;

  badWindows        = 0;
  lastStallCheckMs  = millis();
  lastStallCheckPos = readPosition();
  stateStartMs      = millis();
  state             = WINDING;
  motorDrive(WIND_PWM, true);
  Serial.print(F("Winding to ")); Serial.print(target);
  Serial.print(F(" counts, timeout ")); Serial.print(windTimeoutMs);
  Serial.println(F(" ms"));
}

// ---------------------------------------------------------------------------
// Safety checks
// ---------------------------------------------------------------------------

/* Stuck-motor rule: if the motor is working hard (high current) but the cable
   is not moving, for several checks in a row, return true.
   Call often; it does its own timing. Only valid while the motor is running. */
bool motorIsStalled() {
  unsigned long now = millis();
  if (now - stateStartMs < STARTUP_GRACE_MS) return false;  // spin-up allowance
  if (now - lastStallCheckMs < STALL_WINDOW_MS) return false;

  long pos   = readPosition();
  long moved = abs(pos - lastStallCheckPos);
  bool highCurrent = readCurrent() > STALL_CURRENT_LIMIT;

  if (highCurrent && moved < STALL_MIN_COUNTS) badWindows++;
  else                                         badWindows = 0;

  lastStallCheckMs  = now;
  lastStallCheckPos = pos;
  return badWindows >= STALL_WINDOWS_TO_TRIP;
}

/* Timeout rule: true if the current state has taken longer than allowed. */
bool timedOut(unsigned long limitMs) {
  return (millis() - stateStartMs) > limitMs;
}

/* Stop everything and latch into FAULT until the user resets. */
void enterFault(const char* reason) {
  stopMotor();
  faultReason = reason;
  state = FAULT;
  Serial.print(F("FAULT: ")); Serial.println(reason);
}

// ---------------------------------------------------------------------------
// Main state machine, called every loop()
// ---------------------------------------------------------------------------
void updateWinch() {
  switch (state) {

    case IDLE:
    case HOLDING:
    case FAULT:
      break;                                    // motor stays off

    case WINDING:
      if (readPosition() >= targetCounts - TARGET_TOLERANCE) {
        stopMotor();
        Serial.println(F("Target reached. Waiting for latch..."));
        stateStartMs = millis();
        state = WAIT_LATCH;
      } else if (motorIsStalled()) {
        enterFault("stall: high current, cable not moving");
      } else if (timedOut(windTimeoutMs)) {
        enterFault("timeout: winding took too long");
      }
      break;

    case WAIT_LATCH:
      // Motor is stopped and holding by its gears only. Wait for the latch.
      if (latchClosed()) {
        slackStartPos     = readPosition();
        stateStartMs      = millis();
        lastStallCheckMs  = millis();
        lastStallCheckPos = slackStartPos;
        badWindows        = 0;
        motorDrive(SLACK_PWM, false);           // reverse slowly
        Serial.println(F("Latch closed. Letting out slack..."));
        state = LETTING_OUT;
      } else if (timedOut(LATCH_WAIT_MS)) {
        enterFault("latch did not close in time");
      }
      break;

    case LETTING_OUT:
      if (slackStartPos - readPosition() >= SLACK_COUNTS) {
        stopMotor();
        Serial.println(F("Slack set. Latch now holds the load."));
        state = HOLDING;
      } else if (motorIsStalled()) {
        enterFault("stall while letting out slack");
      } else if (timedOut(SLACK_TIMEOUT_MS)) {
        enterFault("timeout while letting out slack");
      }
      break;
  }
}

// ---------------------------------------------------------------------------
// Serial commands and status printing
// ---------------------------------------------------------------------------

/* Read single-letter commands from the Serial Monitor. */
void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'w': startWinding(WIND_TARGET_COUNTS); break;
      case 'x': stopMotor(); state = IDLE; Serial.println(F("Stopped by user.")); break;
      case 'r':
        if (state == FAULT) { state = IDLE; faultReason = ""; Serial.println(F("Reset to IDLE.")); }
        break;
#if SIMULATE
      case 's': simStall = !simStall; Serial.print(F("simStall=")); Serial.println(simStall); break;
      case 'k': simSlow  = !simSlow;  Serial.print(F("simSlow="));  Serial.println(simSlow);  break;
      case 'l': simLatch = !simLatch; Serial.print(F("simLatch=")); Serial.println(simLatch); break;
#endif
      default: break;
    }
  }
}

/* Print position, current and state about every 200 ms while active. */
void printStatus() {
  static unsigned long lastPrint = 0;
  if (state == IDLE || state == FAULT || state == HOLDING) return;
  if (millis() - lastPrint < 200) return;
  lastPrint = millis();
  Serial.print(stateName(state));
  Serial.print(F("  pos=")); Serial.print(readPosition());
  Serial.print(F("  current=")); Serial.println(readCurrent());
}

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  pinMode(PIN_MOTOR_PWM, OUTPUT);
  pinMode(PIN_MOTOR_DIR, OUTPUT);
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  pinMode(PIN_LATCH, INPUT_PULLUP);
#if !SIMULATE
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encoderISR, RISING);
#endif
  stopMotor();                                  // always start with the motor off
  simLastMs = millis();
  Serial.println(F("Winch ready. Send w to wind, x to stop, r to reset."));
}

void loop() {
  handleSerial();
#if SIMULATE
  updateSimulation();
#endif
  updateWinch();
  printStatus();
}
