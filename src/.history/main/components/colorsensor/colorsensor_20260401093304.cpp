#include <Wire.h>
#include "colorsensor.h"

void ColorSensor::tcaSelect(uint8_t channel) {
  if (channel > 7) return;

  Wire1.beginTransmission(TCA_ADDR);
  Wire1.write(1 << channel);
  Wire1.endTransmission();
}

bool ColorSensor::i2cDetect(uint8_t addr) {
  Wire1.beginTransmission(addr);
  return (Wire1.endTransmission() == 0);
}

void ColorSensor::init() {
  Wire1.begin();

  // Initialize arrays to 0 first
  for (int i = 0; i < NUM_SENSORS; i++) {
    analogValues[i] = 0;
    greenValues[i] = 0;
  }

  // Check mux
  if (!i2cDetect(TCA_ADDR)) {
    Serial.println("TCA9548A not found at 0x70");
    return;
  }

  // Check each sensor channel and start the sensor on that channel
  for (int i = 0; i < NUM_SENSORS; i++) {
    tcaSelect(i);

    if (!i2cDetect(TCS_ADDR)) {
      Serial.print("TCS3472 not found on CH");
      Serial.println(i);
    } else {
      if (!tcs.begin(TCS_ADDR, &Wire1)) {
        Serial.print("TCS3472 failed to start on CH");
        Serial.println(i);
      }
    }
  }

  // Take first readings
  updateReadings();

  // Wait until front sensor gives a nonzero reading
  while (analogValues[0] == 0) {
    updateReadings();
  }

  // Save startup readings as baseline "green field" values
  for (int i = 0; i < NUM_SENSORS; i++) {
    greenValues[i] = analogValues[i];
  }
}

uint16_t* ColorSensor::getAnalogValues() {
  return analogValues;
}

void ColorSensor::updateReadings() {
  uint16_t r, g, b, c;

  for (int i = 0; i < NUM_SENSORS; i++) {
    tcaSelect(i);

    if (!i2cDetect(TCS_ADDR)) {
      analogValues[i] = 0;
      continue;
    }

    tcs.getRawData(&r, &g, &b, &c);

    // Using green only for now
    analogValues[i] = g;
  }
}

void ColorSensor::printReadings() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(i);
    Serial.print(": ");
    Serial.print(analogValues[i]);
    Serial.print("\t");
  }
  Serial.println();
}

void ColorSensor::printGreenValues() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(i);
    Serial.print(": ");
    Serial.print(greenValues[i]);
    Serial.print("\t");
  }
  Serial.println();
}

float ColorSensor::getAvoidAngle() {
  // front = CH0
  if (analogValues[0] >= greenValues[0] + buffer) {
    return 180;
  }

  if (analogValues[1] >= greenValues[1] + buffer) {
    return 135;
  }

  // right = CH2
  if (analogValues[2] >= greenValues[2] + buffer) {
    return 90;
  }

  if (analogValues[3] >= greenValues[3] + buffer) {
    return 45;
  }

  // back = CH4
  if (analogValues[4] >= greenValues[4] + buffer) {
    return 0;
  }

  if (analogValues[5] >= greenValues[5] + buffer) {
    return 315;
  }

  // left = CH6
  if (analogValues[6] >= greenValues[6] + buffer) {
    return 270;
  }

  if (analogValues[7] >= greenValues[7] + buffer) {
    return 225;
  }

  return -1;
}