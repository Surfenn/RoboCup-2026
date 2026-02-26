#ifndef COMPASS_H
#define COMPASS_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BNO055.h"

class Compass {
public:
  void initialize();

  // Returns heading as -180 to +180 degrees.
  // Returns NAN if not initialized.
  float readCompass();

private:
  Adafruit_BNO055 bno{55, 0x29, &Wire};
  bool ok = false;
};

#endif