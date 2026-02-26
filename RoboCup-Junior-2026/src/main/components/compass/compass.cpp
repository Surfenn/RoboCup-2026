#include "compass.h"

void Compass::initialize() {
  Wire.begin();
  delay(10);

  if (!bno.begin()) {
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    ok = false;
    return;              // <-- DO NOT while(1); or your robot will never move
  }

  ok = true;

  bno.setExtCrystalUse(true);
  bno.setMode(OPERATION_MODE_NDOF);
  delay(50);
}

float Compass::readCompass() {
  if (!ok) return NAN;

  sensors_event_t event;
  bno.getEvent(&event);

  float heading = event.orientation.x; // should be 0..360
  // Normalize
  while (heading < 0) heading += 360.0f;
  while (heading >= 360.0f) heading -= 360.0f;

  return heading;
}