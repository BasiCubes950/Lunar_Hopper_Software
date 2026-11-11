
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup() {
  Serial.begin(9600);

  if (!bno.begin()) {
    Serial.println("ERROR: BNO055 not detected!");
    while (1);
  }

  delay(1000);
  bno.setExtCrystalUse(true);

  Serial.println("READY");
}

void loop() {

  if (Serial.available() > 0) {

    char command = Serial.read();

    // Debug message
    if (command == 'a' || command == 'A') {

      Serial.println("START");
      Serial.println("time_ms,orientation_x,orientation_y,orientation_z,accel_x,accel_y,accel_z");

      unsigned long startTime = millis();

      while (millis() - startTime < 10000) {

        sensors_event_t event;
        bno.getEvent(&event);

        imu::Vector<3> acceleration =
            bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

        Serial.print(millis() - startTime);
        Serial.print(",");

        Serial.print(event.orientation.x, 4);
        Serial.print(",");
        Serial.print(event.orientation.y, 4);
        Serial.print(",");
        Serial.print(event.orientation.z, 4);
        Serial.print(",");

        Serial.print(acceleration.x(), 4);
        Serial.print(",");
        Serial.print(acceleration.y(), 4);
        Serial.print(",");
        Serial.println(acceleration.z(), 4);

        delay(100);
      }

      Serial.println("END");
      Serial.println("READY");
    }
  }
}
