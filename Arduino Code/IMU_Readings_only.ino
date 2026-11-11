// imu_readings.ino - BNO055 readings for logging (Lunar Hopper)
//
// Prints one CSV line per reading:
//   time_s, accel_x_mps2, accel_y_mps2, accel_z_mps2, gyro_x_dps, gyro_y_dps, gyro_z_dps
// Units:  time in seconds since the first reading (0.0100 s steps = 100 Hz),
//         acceleration in m/s^2 (flat and still: z is about +9.81),
//         gyro rate in degrees per second (still: all close to 0).
//
// Wiring: BNO055 VIN->5V, GND->GND, SDA->A4, SCL->A5.
// Serial: 115200 baud. Use with imu_log_to_github.py, or view in the Serial Monitor / Plotter.

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// ---------- settings ----------
// The BNO055 updates its data at 100 Hz (every 10 ms); reading faster only repeats values.
const unsigned long SAMPLE_INTERVAL_US = 10000;   // 10 ms = 100 readings per second
// AMG = raw accelerometer + gyro, no fusion: shows the sensor's real bias and drift (Lab 3 tests).
// IMUPLUS = the chip's own accel+gyro fusion and self-calibration (what the hopper code uses).
const adafruit_bno055_opmode_t IMU_MODE = OPERATION_MODE_AMG;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
bool sensorOk = false;
unsigned long nextSampleUs = 0, startUs = 0, lastRetryMs = 0;

bool startSensor() {
  if (!bno.begin(IMU_MODE)) return false;
  bno.setExtCrystalUse(true);
  return true;
}

void printHeader() {
  Serial.println(F("time_s,accel_x_mps2,accel_y_mps2,accel_z_mps2,gyro_x_dps,gyro_y_dps,gyro_z_dps"));
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("# imu_readings: time [s], acceleration [m/s^2], gyro rate [deg/s]"));
  sensorOk = startSensor();
  if (sensorOk) {
    Serial.println(F("# BNO055 found"));
    printHeader();
    startUs = micros();
    nextSampleUs = startUs;
  } else {
    Serial.println(F("# ERROR: BNO055 not found. Check VIN/GND/SDA(A4)/SCL(A5). Retrying every second."));
  }
}

void loop() {
  unsigned long now = millis();

  if (!sensorOk) {                              // no while(1): keep retrying instead of freezing
    if (now - lastRetryMs >= 1000) {
      lastRetryMs = now;
      sensorOk = startSensor();
      if (sensorOk) { Serial.println(F("# BNO055 found")); printHeader(); startUs = micros(); nextSampleUs = startUs; }
      else Serial.println(F("# still no BNO055: check VIN->5V, GND, SDA->A4, SCL->A5"));
    }
    return;
  }

  // Fixed 10 ms schedule: the next sample time advances by exactly 10 ms each time,
  // so small delays don't add up (no drift, no delay()).
  unsigned long nowUs = micros();
  if ((long)(nowUs - nextSampleUs) < 0) return;   // not time yet
  nextSampleUs += SAMPLE_INTERVAL_US;
  if ((long)(nowUs - nextSampleUs) > (long)SAMPLE_INTERVAL_US) nextSampleUs = nowUs;  // fell far behind: resync

  imu::Vector<3> acc  = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER); // m/s^2
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);     // deg/s

  Serial.print((nowUs - startUs) / 1000000.0, 4);  Serial.print(',');   // seconds since the first reading
  Serial.print(acc.x(), 3);       Serial.print(',');
  Serial.print(acc.y(), 3);       Serial.print(',');
  Serial.print(acc.z(), 3);       Serial.print(',');
  Serial.print(gyro.x(), 3);      Serial.print(',');
  Serial.print(gyro.y(), 3);      Serial.print(',');
  Serial.println(gyro.z(), 3);
}
