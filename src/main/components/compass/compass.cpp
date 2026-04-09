#include "compass.h"

void Compass::initialize() {
  Wire1.begin();
  if (!bno.begin()) {
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    return;
  }
  bno.setExtCrystalUse(true);
  bno.setMode(OPERATION_MODE_NDOF);
  delay(500);

  // Clear zero-offset initially so we read absolute physical compass degree
  initialOffset = 0.0;

  // Let the compass warm up and discard the first few jittery frames
  for (int i = 0; i < 20; i++) {
    readCompass();
    delay(20);
  }

  // Snapshot the exact heading the robot was facing when booted
  initialOffset = readCompass();

  // uint8_t system, gyro, accel, mag;
  // bno.getCalibration(&system, &gyro, &accel, &mag);
  // Serial.print("CALIBRATION: Sys=");
  // Serial.print(system);
  // Serial.print(" Gyro=");
  // Serial.print(gyro);
  // Serial.print(" Accel=");
  // Serial.print(accel);
  // Serial.print(" Mag=");
  // Serial.println(mag);
}

float Compass::readCompass() {

  sensors_event_t event;
  bno.getEvent(&event);
  float angle = event.orientation.x;

  // Re-zero the compass to whatever direction the robot was initialized in
  angle -= initialOffset;
  if (angle < 0.0) {
    angle += 360.0;
  }

  return angle;
}