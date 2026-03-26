#ifndef COLORSENSOR_H          // Include guard start → prevents multiple inclusion of this header file
#define COLORSENSOR_H

#include <Arduino.h>          // Includes Arduino types (e.g., uint8_t, Serial, etc.)

// ColorSensor class handles multiple color sensors (via mux) for line detection
class ColorSensor {
public:
  void init();                // Initializes I2C, mux, and all color sensors, and stores baseline values

  void updateReadings();      // Updates the current readings from all sensors

  // Returns pointer to raw values (size = 8)
  uint16_t* getAnalogValues(); // Gives access to the latest sensor readings array

  void printReadings();       // Prints current sensor readings to Serial for debugging
  void printGreenValues();    // Prints baseline values (initial readings) for debugging

  // Returns angle to move AWAY from detected line, or -1 if no line
  float getAvoidAngle();      // Computes and returns avoidance direction based on detected line

public:
  static const uint8_t NUM_SENSORS = 8;  // Number of sensors connected (via mux channels 0–7)
};

#endif                        // End of include guard