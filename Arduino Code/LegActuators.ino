#include <Servo.h>

// ===================== CONFIGURATION =====================
const uint8_t NUM_LEGS = 3;
const uint8_t LEG_PINS[NUM_LEGS]      = {9, 10, 11};   // TODO: actual PWM pins
const int8_t  FEEDBACK_PINS[NUM_LEGS] = {A0, A1, A2};  // TODO: actual analog pins, or -1 if none

const float STROKE_MM      = 100.0f;  // L16-100 full stroke
const float MAX_SPEED_MM_S = 13.0f;   // *** ESTIMATE *** TODO: measure in person and replace

// Feedback calibration: analogRead() value at each end of the stroke
const int ADC_AT_0_MM      = 0;       // TODO: calibrate
const int ADC_AT_STROKE_MM = 1023;    // TODO: calibrate

// Ground detection tuning (all need testing on the real hopper)
const float         REACHED_TOL_MM  = 1.5f;   // within this of target = reached
const float         STALL_EPS_MM    = 0.5f;   // moved less than this over the window = stalled
const unsigned long STALL_WINDOW_MS = 300;    // how long it must be stalled
const unsigned long START_GRACE_MS  = 400;    // ignore stalls right after a new command

// Folding while lowering to jump (all estimates, tune on the real hopper)
const float LAUNCH_POSE_MM = 80.0f;   // TODO: leg length in the down/tripod launch pose
const float FOLD_MM_PER_MM = -0.5f;   // leg change per mm the hopper lowers; sign = fold direction
const float MAX_FOLD_MM    = 25.0f;   // never fold more than this
// =========================================================

// Leg status values (plain constants so they work with Arduino's auto-prototypes)
const uint8_t LEG_IDLE    = 0;
const uint8_t LEG_MOVING  = 1;
const uint8_t LEG_REACHED = 2;
const uint8_t LEG_BLOCKED = 3;   // stalled short of target = touching ground

class Leg {
  public:
    void begin(uint8_t pin, int8_t fbPin) {
      m_fbPin  = fbPin;
      m_servo.attach(pin, 1000, 2000);
      m_target = -100.0f;                 // invalid, so the first moveTo always runs
      m_lastMs = millis();
      if (hasFeedback()) {
        m_meas = readMeasuredMm();
        m_est  = m_meas;
      } else {
        m_est  = STROKE_MM;               // unknown start: assume fully extended
      }
      moveTo(0);                          // home to retracted
    }

    void moveTo(float mm) {
      float t = constrain(mm, 0.0f, STROKE_MM);
      // Repeated calls with the same target (e.g. fold called every loop) must not
      // restart the move, or stall detection would never get a chance to trigger.
      if (m_status != LEG_BLOCKED && m_status != LEG_IDLE && fabs(t - m_target) < 0.5f) return;
      m_target = t;
      writeTarget();
      m_status      = LEG_MOVING;
      m_moveStartMs = millis();
      m_stallRefPos = position();
      m_stallRefMs  = m_moveStartMs;
    }

    void update() {
      unsigned long now = millis();
      float step = MAX_SPEED_MM_S * (now - m_lastMs) / 1000.0f;
      m_lastMs = now;

      // Estimated position (speed model)
      if (m_est > m_target + step)      m_est -= step;
      else if (m_est < m_target - step) m_est += step;
      else                              m_est = m_target;

      // Measured position (low-pass filtered to reduce ADC noise)
      if (hasFeedback()) m_meas = 0.7f * m_meas + 0.3f * readMeasuredMm();

      if (m_status != LEG_MOVING) return;

      float pos = position();
      if (fabs(m_target - pos) <= REACHED_TOL_MM) {
        m_status = LEG_REACHED;
        return;
      }

      // Ground detection: not at target, and no longer making progress
      if (!hasFeedback()) return;
      if (now - m_moveStartMs < START_GRACE_MS) return;
      if (fabs(pos - m_stallRefPos) > STALL_EPS_MM) {
        m_stallRefPos = pos;
        m_stallRefMs  = now;
      } else if (now - m_stallRefMs >= STALL_WINDOW_MS) {
        m_status = LEG_BLOCKED;
        m_target = pos;                   // stop pushing into the ground
        m_est    = pos;
        writeTarget();
      }
    }

    bool    hasFeedback() const { return m_fbPin >= 0; }
    float   position()    const { return hasFeedback() ? m_meas : m_est; }
    float   target()      const { return m_target; }
    uint8_t status()      const { return m_status; }

  private:
    float readMeasuredMm() const {
      int raw = analogRead(m_fbPin);
      float mm = (float)(raw - ADC_AT_0_MM) * STROKE_MM / (float)(ADC_AT_STROKE_MM - ADC_AT_0_MM);
      return constrain(mm, 0.0f, STROKE_MM);
    }
    void writeTarget() {
      m_servo.writeMicroseconds(1000 + (int)(m_target / STROKE_MM * 1000.0f));
    }

    Servo   m_servo;
    int8_t  m_fbPin = -1;
    float   m_target = 0.0f, m_est = 0.0f, m_meas = 0.0f;
    uint8_t m_status = LEG_IDLE;
    float   m_stallRefPos = 0.0f;
    unsigned long m_lastMs = 0, m_moveStartMs = 0, m_stallRefMs = 0;
};

Leg legs[NUM_LEGS];

// ---------------- Public functions (this is what D2 calls) ----------------

void moveLegTo(uint8_t i, float mm) {            // absolute, 0-100 mm
  if (i < NUM_LEGS) legs[i].moveTo(mm);
}

void moveLegBy(uint8_t i, float deltaMm) {       // relative to current target
  if (i < NUM_LEGS) legs[i].moveTo(legs[i].target() + deltaMm);
}

float getLegPosition(uint8_t i) {                // -1 if bad index
  return (i < NUM_LEGS) ? legs[i].position() : -1.0f;
}

uint8_t getLegStatus(uint8_t i) {                // LEG_IDLE/MOVING/REACHED/BLOCKED
  return (i < NUM_LEGS) ? legs[i].status() : LEG_IDLE;
}

bool legIsDone(uint8_t i) {                      // reached or blocked
  if (i >= NUM_LEGS) return true;
  uint8_t s = legs[i].status();
  return s == LEG_REACHED || s == LEG_BLOCKED;
}

bool allLegsDone() {
  for (uint8_t i = 0; i < NUM_LEGS; i++) if (!legIsDone(i)) return false;
  return true;
}

bool legIsTouchingGround(uint8_t i) {            // only meaningful with feedback
  return (i < NUM_LEGS) && legs[i].status() == LEG_BLOCKED;
}

bool legHasFeedback(uint8_t i) {
  return (i < NUM_LEGS) && legs[i].hasFeedback();
}

// Call repeatedly while the hopper lowers to jump.
// compressionMm = how far the hopper has lowered (from the winch or other source).
// Legs fold a little so they stay on the ground without blocking the fibers bending.
void foldLegsForJump(float compressionMm) {
  float fold = constrain(compressionMm * FOLD_MM_PER_MM, -MAX_FOLD_MM, MAX_FOLD_MM);
  for (uint8_t i = 0; i < NUM_LEGS; i++) moveLegTo(i, LAUNCH_POSE_MM + fold);
}

void updateLegs() {                              // call EVERY loop
  for (uint8_t i = 0; i < NUM_LEGS; i++) legs[i].update();
}

void beginLegs() {                               // call once from setup()
  for (uint8_t i = 0; i < NUM_LEGS; i++) legs[i].begin(LEG_PINS[i], FEEDBACK_PINS[i]);
}
