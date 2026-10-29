#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// all TODO: replace from CAD / testing
// Angle of each leg around the hopper, measured from the IMU +x axis, same sense as atan2(y, x).
// Evenly spaced is known; the starting offset is a placeholder.
const float LEG_AZIMUTH_DEG[NUM_LEGS] = {0.0f, 120.0f, 240.0f};

const float LEG_PUSH_SIGN        = 1.0f;   // +1 if a LONGER leg pushes the body away from the ground
const float RIGHTING_NEUTRAL_MM  = 50.0f;  // leg length when there is no tilt (mid-stroke)
const float MM_PER_RAD           = 40.0f;  // ESTIMATE: leg length change per radian of tilt
const float MAX_PUSH_MM          = 50.0f;  // largest change from neutral
const float DEFAULT_PUSH_AZ_DEG  = 0.0f;   // direction to push if exactly upside down (ambiguous)

const float TILT_DONE_DEG        = 8.0f;   // at or below this tilt = upright
const float AUTO_TRIGGER_TILT_DEG = 15.0f; // still AND tilted more than this = start self-righting
                                           // (bigger than TILT_DONE_DEG so it can't re-trigger on noise)
const bool  AUTO_SELF_RIGHTING   = true;   // false = only start with startSelfRighting()
const float STILL_ACCEL_MS2      = 0.3f;   // linear accel below this = still
const unsigned long STILL_TIME_MS   = 500; // must stay still this long
const unsigned long MOVE_TIMEOUT_MS = 8000;
const uint8_t MAX_ATTEMPTS       = 10;     // give up after this many correction moves

// #define D2_DEBUG                        // uncomment for Serial prints
// =============================================================================================

const float DEG2RAD_D2 = 0.0174532925f;
const float RAD2DEG_D2 = 57.2957795f;

// States (plain constants so they work with Arduino's auto-prototypes)
const uint8_t D2_IDLE       = 0;
const uint8_t D2_WAIT_STILL = 1;
const uint8_t D2_DECIDE     = 2;
const uint8_t D2_MOVING     = 3;
const uint8_t D2_FINISHING  = 4;   // upright; legs returning to launch pose
const uint8_t D2_DONE       = 5;   // ready to reload for the jump
const uint8_t D2_FAILED     = 6;

// IMUPLUS = accel + gyro fusion, no magnetometer (motors and magnets nearby would disturb it)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

uint8_t d2State = D2_IDLE;
bool    monStill = false;
unsigned long monStillStart = 0;
uint8_t d2Attempts = 0;
bool    d2Still = false;
unsigned long d2StillStart = 0, d2MoveStart = 0;

bool selfRightingBegin() {                       // call once from setup()
  return bno.begin(OPERATION_MODE_IMUPLUS);
}

// Unit "up" vector in the hopper's own frame: (0,0,1) when upright.
// ASSUMES the IMU is mounted with +z pointing up when the hopper is upright.
// TODO: if it is mounted differently, remap the axes HERE (keep it a proper rotation,
// not just a single sign flip).
bool readUpVector(float &ux, float &uy, float &uz) {
  imu::Vector<3> g = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  float n = sqrtf(g.x() * g.x() + g.y() * g.y() + g.z() * g.z());
  if (n < 1.0f) return false;                    // bad read
  ux = g.x() / n;  uy = g.y() / n;  uz = g.z() / n;
  return true;
}

// The decision: orientation in, one target length per leg out.
//   tilt   = angle between hopper's up axis and real up (0 = upright, 180 = upside down)
//   Legs on the side of the hopper facing the ground extend (push); legs on the opposite
//   side retract. The push gets bigger with tilt and fades to neutral as it nears upright.
void computeLegTargets(float ux, float uy, float uz, float *targets) {
  float tilt  = acosf(constrain(uz, -1.0f, 1.0f));
  float horiz = sqrtf(ux * ux + uy * uy);
  // "Up" leans toward (ux,uy), so the ground-facing side is the opposite direction.
  float groundAz = (horiz < 0.05f) ? DEFAULT_PUSH_AZ_DEG * DEG2RAD_D2 : atan2f(-uy, -ux);
  float amp = min(MAX_PUSH_MM, MM_PER_RAD * tilt);
  for (uint8_t i = 0; i < NUM_LEGS; i++) {
    float w = cosf(LEG_AZIMUTH_DEG[i] * DEG2RAD_D2 - groundAz);   // +1 ground side ... -1 far side
    targets[i] = constrain(RIGHTING_NEUTRAL_MM + LEG_PUSH_SIGN * amp * w, 0.0f, STROKE_MM);
  }
}

void startSelfRighting() {
  d2Attempts = 0;
  d2Still = false;
  d2State = D2_WAIT_STILL;
}

uint8_t selfRightingState()  { return d2State; }
bool    selfRightingDone()   { return d2State == D2_DONE; }
bool    selfRightingFailed() { return d2State == D2_FAILED; }

void updateSelfRighting() {                      // call EVERY loop
  unsigned long now = millis();
  switch (d2State) {

    case D2_WAIT_STILL: {
      imu::Vector<3> a = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
      if (a.magnitude() < STILL_ACCEL_MS2) {
        if (!d2Still) { d2Still = true; d2StillStart = now; }
        if (now - d2StillStart >= STILL_TIME_MS) { d2Still = false; d2State = D2_DECIDE; }
      } else {
        d2Still = false;
      }
      break;
    }

    case D2_DECIDE: {
      float ux, uy, uz;
      if (!readUpVector(ux, uy, uz)) { d2State = D2_WAIT_STILL; break; }
      float tiltDeg = acosf(constrain(uz, -1.0f, 1.0f)) * RAD2DEG_D2;
#ifdef D2_DEBUG
      Serial.print("D2 tilt(deg)="); Serial.println(tiltDeg);
#endif
      if (tiltDeg <= TILT_DONE_DEG) {            // upright: legs back to launch pose
        for (uint8_t i = 0; i < NUM_LEGS; i++) moveLegTo(i, LAUNCH_POSE_MM);
        d2MoveStart = now;
        d2State = D2_FINISHING;
        break;
      }
      if (d2Attempts >= MAX_ATTEMPTS) { d2State = D2_FAILED; break; }
      float targets[NUM_LEGS];
      computeLegTargets(ux, uy, uz, targets);
      for (uint8_t i = 0; i < NUM_LEGS; i++) {
        moveLegTo(i, targets[i]);
#ifdef D2_DEBUG
        Serial.print("  leg "); Serial.print(i); Serial.print(" -> "); Serial.println(targets[i]);
#endif
      }
      d2Attempts++;
      d2MoveStart = now;
      d2State = D2_MOVING;
      break;
    }

    case D2_MOVING:                              // legs moving, hopper rotating
      if (allLegsDone() || now - d2MoveStart > MOVE_TIMEOUT_MS) d2State = D2_WAIT_STILL;
      break;

    case D2_FINISHING:
      if (allLegsDone() || now - d2MoveStart > MOVE_TIMEOUT_MS) d2State = D2_DONE;
      break;

    default:                                     // IDLE, DONE, FAILED: nothing to do
      break;
  }
}

// Call EVERY loop. Starts self-righting automatically when the hopper has been still for
// STILL_TIME_MS while tilted more than AUTO_TRIGGER_TILT_DEG. Only watches while D2 is idle
// or done; it will not restart after a FAILED attempt.
void checkAutoSelfRighting() {
  if (!AUTO_SELF_RIGHTING) return;
  if (d2State != D2_IDLE && d2State != D2_DONE) { monStill = false; return; }
  unsigned long now = millis();
  imu::Vector<3> a = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  if (a.magnitude() >= STILL_ACCEL_MS2) { monStill = false; return; }
  if (!monStill) { monStill = true; monStillStart = now; }
  if (now - monStillStart < STILL_TIME_MS) return;
  float ux, uy, uz;
  if (!readUpVector(ux, uy, uz)) return;
  if (acosf(constrain(uz, -1.0f, 1.0f)) * RAD2DEG_D2 > AUTO_TRIGGER_TILT_DEG) {
    monStill = false;
    startSelfRighting();
  }
}
