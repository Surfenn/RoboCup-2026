#ifndef COLORSENSOR_H
#define COLORSENSOR_H

#include <Wire.h>
#include "Adafruit_TCS34725.h"

class ColorSensor {
  public:
    void init();
    void printReadings();
    void printGreenValues();
    void updateReadings();
    float getWarningAvoidAngle();
    float getAvoidAngle();
    uint16_t* getAnalogValues();

  private:
    static const uint8_t TCA_ADDR = 0x70;
    static const uint8_t TCS_ADDR = 0x29;
    static const uint8_t numChannels = 8;   // TCA9548A has 8 channels
    int buffer = 150;

    uint16_t analogValues[numChannels];
    uint16_t greenValues[numChannels];

    Adafruit_TCS34725 tcs = Adafruit_TCS34725(
      TCS34725_INTEGRATIONTIME_50MS,
      TCS34725_GAIN_4X
    );

    void tcaSelect(uint8_t channel);
    bool i2cDetect(uint8_t addr);
};

#endif