#ifndef COLORSENSOR_H
#define COLORSENSOR_H

#include <Arduino.h>

class ColorSensor {
public:
  void init();
  void updateReadings();

  // Returns pointer to raw values (size = 8)
  uint16_t* getAnalogValues();

  void printReadings();
  void printGreenValues();

  // Returns angle to move AWAY from detected line, or -1 if no line
  float getAvoidAngle();

public:
  static const uint8_t NUM_SENSORS = 8;
};

#endif