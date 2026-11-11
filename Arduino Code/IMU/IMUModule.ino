// D5: IMU module (BNO055). This file is the ONLY place that talks to the BNO055;
// SelfRighting.ino (D2) asks it for tilt / up vector / stillness.
// Main program: imuBegin() once in setup(), imuUpdate() every loop, then ask:
//   imuTiltDeg()        how far the hopper is leaning (0 upright, 90 on its side, 180 upside down)
//   imuIsFalling()      in the air (free fall)
//   imuImpactDetected() just hit the ground (true ONCE per impact)
//   imuIsStill()        stopped moving
//   imuHasFault()       sensor missing or lost -> main program must stop the motors
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// ===================== CONFIGURATION (all TODO: tune in person) =====================
#define IMU_SIMULATE   // pretend values, no sensor needed. Comment out once the BNO055 is wired.
#define IMU_DEBUG      // print IMU status to Serial every IMU_PRINT_INTERVAL_MS

const unsigned long IMU_READ_INTERVAL_MS  = 10;   // 100 Hz: fast enough to catch a landing spike
const unsigned long IMU_PRINT_INTERVAL_MS = 250;

// Falling: in free fall the accelerometer reads close to 0
const float         FALL_ACCEL_MS2    = 3.0f;     // total accel below this = weightless
const unsigned long FALL_TIME_MS      = 60;       // must last this long (ignores single noisy samples)

// Impact: spike in total accel. Fusion mode fixes the range at +/-4 g (39 m/s^2): keep below that.
const float         IMPACT_ACCEL_MS2  = 30.0f;
const unsigned long IMPACT_LOCKOUT_MS = 300;      // ignore bounces right after an impact

// Still: linear accel (gravity removed) and rotation both small, for a while
const float         IMU_STILL_ACCEL_MS2 = 0.3f;   // was STILL_ACCEL_MS2 in SelfRighting.ino
const float         IMU_STILL_GYRO_DPS  = 3.0f;
const unsigned long IMU_STILL_TIME_MS   = 500;    // was STILL_TIME_MS in SelfRighting.ino

// Sensor lost: this many bad reads in a row (all zeros) = fault
const uint8_t       IMU_MAX_BAD_READS = 20;       // 20 x 10 ms = 0.2 s
// =====================================================================================

const float IMU_RAD2DEG = 57.2957795f;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

bool    imuFault = false;
uint8_t imuBadReads = 0;
unsigned long imuLastReadMs = 0, imuLastPrintMs = 0;

float  imuUp[3] = {0.0f, 0.0f, 1.0f};            // unit "up" in the hopper's frame
float  imuTilt = 0.0f;
float  imuAccMag = 9.81f, imuLinMag = 0.0f, imuGyroMag = 0.0f;

bool   imuFallCand = false, imuFalling = false;
unsigned long imuFallStart = 0;

bool   imuImpactPending = false;
unsigned int  imuImpactCount = 0;
unsigned long imuLastImpactMs = 0;

bool   imuStillCand = false;
unsigned long imuStillStart = 0;

// Vectors are float[3] = {x, y, z} (plain arrays: Arduino's auto-prototypes break on custom structs)
float imuMag(const float *v) { return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); }

void imuSet(float *v, float x, float y, float z) { v[0] = x; v[1] = y; v[2] = z; }

// ---------------- Pretend values (IMU_SIMULATE) ----------------
#ifdef IMU_SIMULATE
const uint8_t SIM_STILL   = 0;
const uint8_t SIM_MOVING  = 1;
const uint8_t SIM_FALLING = 2;
uint8_t simMode = SIM_STILL;
float   simTiltDeg = 0.0f;
unsigned long simImpactUntil = 0;

float simNoise(float amp) { return amp * (float)random(-1000, 1001) / 1000.0f; }

bool imuReadSensor(float *acc, float *grav, float *gyro) {
  float t = simTiltDeg / IMU_RAD2DEG;            // lean around the x axis
  imuSet(grav, 0.0f, 9.81f * sinf(t), 9.81f * cosf(t));
  for (uint8_t i = 0; i < 3; i++) {
    acc[i]  = grav[i] + simNoise(0.05f);
    gyro[i] = simNoise(0.2f);
    if (simMode == SIM_MOVING)  { acc[i] += simNoise(2.0f); gyro[i] += simNoise(40.0f); }
    if (simMode == SIM_FALLING) { acc[i]  = simNoise(0.3f); gyro[i]  = simNoise(5.0f);  }
  }
  if (millis() < simImpactUntil) acc[2] += 28.0f; // landing spike, ~38 m/s^2 total
  return true;
}

// Keys (capital letters, so they don't clash with the leg keys r/f/h/0):
//   U upright   T tilted 30   S on its side   D upside down   M moving   F falling   L land (impact)
void imuSimCommand(char c) {
  switch (c) {
    case 'U': simMode = SIM_STILL; simTiltDeg = 0.0f;   break;
    case 'T': simMode = SIM_STILL; simTiltDeg = 30.0f;  break;
    case 'S': simMode = SIM_STILL; simTiltDeg = 90.0f;  break;
    case 'D': simMode = SIM_STILL; simTiltDeg = 180.0f; break;
    case 'M': simMode = SIM_MOVING;  break;
    case 'F': simMode = SIM_FALLING; break;
    case 'L': simMode = SIM_STILL; simImpactUntil = millis() + 20; break;
    default:  return;
  }
  Serial.print(F("IMU sim -> ")); Serial.println(c);
}

#else
// ---------------- Real BNO055 ----------------
bool imuReadSensor(float *acc, float *grav, float *gyro) {
  imu::Vector<3> a = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);  // m/s^2
  imu::Vector<3> g = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);        // m/s^2
  imu::Vector<3> w = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);      // deg/s
  // TODO: if the IMU is not mounted with +z up when the hopper is upright, remap the axes HERE
  // (a proper rotation, not just one sign flip). Everything else uses these vectors.
  imuSet(acc,  a.x(), a.y(), a.z());
  imuSet(grav, g.x(), g.y(), g.z());
  imuSet(gyro, w.x(), w.y(), w.z());
  // A disconnected sensor reads all zeros
  return imuMag(acc) > 0.0f || imuMag(grav) > 0.0f;
}

void imuSimCommand(char c) { (void)c; }          // nothing to simulate
#endif

// ---------------- Setup / update ----------------
bool imuBegin() {                                // call once from setup()
#ifdef IMU_SIMULATE
  imuFault = false;
  Serial.println(F("IMU: SIMULATION (pretend values). Keys U T S D M F L"));
#else
  // IMUPLUS = accel + gyro fusion, no magnetometer (motors and magnets nearby would disturb it)
  imuFault = !bno.begin(OPERATION_MODE_IMUPLUS); // replaces while(1): no freeze, just a fault flag
  if (!imuFault) bno.setExtCrystalUse(true);
#endif
  imuLastReadMs = millis();
  return !imuFault;
}

void imuPrintStatus() {
  Serial.print(F("IMU tilt="));    Serial.print(imuTilt, 1);
  Serial.print(F(" falling="));    Serial.print(imuFalling);
  Serial.print(F(" impacts="));    Serial.print(imuImpactCount);
  Serial.print(F(" still="));      Serial.print(imuIsStill());
  Serial.print(F(" fault="));      Serial.print(imuFault);
  Serial.print(F("  |acc|="));     Serial.print(imuAccMag, 2);
  Serial.print(F(" |lin|="));      Serial.print(imuLinMag, 2);
  Serial.print(F(" |gyro|="));     Serial.println(imuGyroMag, 1);
}

void imuUpdate() {                               // call EVERY loop; returns at once if not time yet
  unsigned long now = millis();
#ifdef IMU_DEBUG
  if (now - imuLastPrintMs >= IMU_PRINT_INTERVAL_MS) { imuLastPrintMs = now; imuPrintStatus(); }
#endif
  if (imuFault) return;
  if (now - imuLastReadMs < IMU_READ_INTERVAL_MS) return;
  imuLastReadMs = now;

  float acc[3], grav[3], gyro[3];
  if (!imuReadSensor(acc, grav, gyro)) {
    if (++imuBadReads >= IMU_MAX_BAD_READS) {
      imuFault = true;
      imuFalling = false; imuStillCand = false;
    }
    return;
  }
  imuBadReads = 0;

  // Tilt from the gravity direction (works at any angle, unlike the X/Y/Z angles)
  float gn = imuMag(grav);
  if (gn > 1.0f) {
    imuSet(imuUp, grav[0] / gn, grav[1] / gn, grav[2] / gn);
    imuTilt = acosf(constrain(imuUp[2], -1.0f, 1.0f)) * IMU_RAD2DEG;
  }

  float lin[3] = {acc[0] - grav[0], acc[1] - grav[1], acc[2] - grav[2]};
  imuAccMag  = imuMag(acc);
  imuLinMag  = imuMag(lin);
  imuGyroMag = imuMag(gyro);

  // Falling
  if (imuAccMag < FALL_ACCEL_MS2) {
    if (!imuFallCand) { imuFallCand = true; imuFallStart = now; }
    imuFalling = (now - imuFallStart >= FALL_TIME_MS);
  } else {
    imuFallCand = false; imuFalling = false;
  }

  // Impact
  if (imuAccMag > IMPACT_ACCEL_MS2 &&
      (imuImpactCount == 0 || now - imuLastImpactMs > IMPACT_LOCKOUT_MS)) {
    imuImpactPending = true;
    imuImpactCount++;
    imuLastImpactMs = now;
  }

  // Still
  if (imuLinMag < IMU_STILL_ACCEL_MS2 && imuGyroMag < IMU_STILL_GYRO_DPS) {
    if (!imuStillCand) { imuStillCand = true; imuStillStart = now; }
  } else {
    imuStillCand = false;
  }
}

// ---------------- Public functions ----------------
float imuTiltDeg()     { return imuTilt; }
bool  imuIsFalling()   { return !imuFault && imuFalling; }
bool  imuHasFault()    { return imuFault; }

bool imuImpactDetected() {                       // true ONCE per impact, then clears
  bool p = imuImpactPending;
  imuImpactPending = false;
  return p;
}

unsigned long imuStillDurationMs() {             // how long it has been still (0 = moving)
  return (!imuFault && imuStillCand) ? millis() - imuStillStart : 0;
}

bool imuIsStill() { return imuStillDurationMs() >= IMU_STILL_TIME_MS; }

void imuGetUpVector(float &ux, float &uy, float &uz) {  // (0,0,1) when upright; used by D2
  ux = imuUp[0]; uy = imuUp[1]; uz = imuUp[2];
}
