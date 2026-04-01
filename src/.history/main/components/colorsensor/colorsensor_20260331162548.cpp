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

  Serial.println("=== COLOR SENSOR INIT ===");

  // Check mux
  Serial.print("Checking for TCA9548A at 0x70... ");
  if (!i2cDetect(TCA_ADDR)) {
    Serial.println("NOT FOUND!");
    while (1) {}
  }
  Serial.println("FOUND!");

  // Initialize all arrays to 0 first
  for (int i = 0; i < numChannels; i++) {
    analogValues[i] = 0;
    greenValues[i] = 0;
  }

  // Start sensor on first detected channel
  bool sensorStarted = false;

  for (int ch = 0; ch < numChannels; ch++) {
    tcaSelect(ch);
    delay(10);

    Serial.print("Checking for TCS34725 at 0x29 on CH");
    Serial.print(ch);
    Serial.print("... ");

    if (i2cDetect(TCS_ADDR)) {
      Serial.println("FOUND!");

      if (!sensorStarted) {
        if (!tcs.begin(TCS_ADDR, &Wire1)) {
          Serial.println("ERROR: Sensor detected but library begin() failed.");
          while (1) {}
        }
        sensorStarted = true;
      }
    } else {
      Serial.println("NOT FOUND");
    }
  }

  if (!sensorStarted) {
    Serial.println("No TCS34725 sensors detected on any mux channel.");
    while (1) {}
  }

  // Take initial baseline readings
  updateReadings();

  for (int i = 0; i < numChannels; i++) {
    greenValues[i] = analogValues[i];
  }

  Serial.println("Color sensors ready!");
}

uint16_t* ColorSensor::getAnalogValues() {
  return analogValues;
}

void ColorSensor::updateReadings() {
  for (int ch = 0; ch < numChannels; ch++) {
    tcaSelect(ch);
    delay(2);

    if (i2cDetect(TCS_ADDR)) {
      uint16_t r, g, b, c;
      tcs.getRawData(&r, &g, &b, &c);

      // Store GREEN channel like your old code was doing
      analogValues[ch] = g;
    } else {
      analogValues[ch] = 0;
    }
  }
}

void ColorSensor::printReadings() {
  for (int i = 0; i < numChannels; i++) {
    Serial.print(i);
    Serial.print(": ");
    Serial.print(analogValues[i]);
    Serial.print("\t");
  }
  Serial.println();
}

void ColorSensor::printGreenValues() {
  for (int i = 0; i < numChannels; i++) {
    Serial.print(i);
    Serial.print(": ");
    Serial.print(greenValues[i]);
    Serial.print("\t");
  }
  Serial.println();
}

float ColorSensor::getAvoidAngle() {
  // For 8 channels:
  // 0 = front
  // 2 = left
  // 4 = back
  // 6 = right

  // front
  if (analogValues[0] >= greenValues[0] + buffer) {
    return 180;
  }

  // left
  if (analogValues[2] >= greenValues[2] + buffer) {
    return 90;
  }

  // back
  if (analogValues[4] >= greenValues[4] + buffer) {
    return 0;
  }

  // right
  if (analogValues[6] >= greenValues[6] + buffer) {
    return 270;
  }

  return -1;
}

float ColorSensor::getWarningAvoidAngle() {
  return getAvoidAngle();
}