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
  delay(50);

  Serial.println("=== COLOR SENSOR INIT ===");

  Serial.print("Checking for TCA9548A at 0x70... ");
  if (!i2cDetect(TCA_ADDR)) {
    Serial.println("NOT FOUND!");
    while (1) {}
  }
  Serial.println("FOUND!");

  for (int i = 0; i < NUM_SENSORS; i++) {
    analogValues[i] = 0;
    greenValues[i] = 0;
  }

  bool foundAny = false;

  for (int ch = 0; ch < NUM_SENSORS; ch++) {
    tcaSelect(ch);
    delay(10);

    Serial.print("Checking for TCS3472 at 0x29 on CH");
    Serial.print(ch);
    Serial.print("... ");

    if (i2cDetect(TCS_ADDR)) {
      Serial.println("FOUND");
      foundAny = true;
    } else {
      Serial.println("NOT FOUND");
    }
  }

  if (!foundAny) {
    Serial.println("No TCS3472 sensors found on any mux channel.");
    while (1) {}
  }

  // Initialize library on channel 0 first that has a sensor
  for (int ch = 0; ch < NUM_SENSORS; ch++) {
    tcaSelect(ch);
    delay(10);

    if (i2cDetect(TCS_ADDR)) {
      if (!tcs.begin(TCS_ADDR, &Wire1)) {
        Serial.print("Library begin failed on CH");
        Serial.println(ch);
        while (1) {}
      }

      Serial.print("TCS3472 started on CH");
      Serial.println(ch);
      break;
    }
  }

  delay(100);
  updateReadings();

  for (int i = 0; i < NUM_SENSORS; i++) {
    greenValues[i] = analogValues[i];
  }

  Serial.println("Color sensor init complete.");
}

uint16_t* ColorSensor::getAnalogValues() {
  return analogValues;
}

void ColorSensor::updateReadings() {
  for (int ch = 0; ch < NUM_SENSORS; ch++) {
    tcaSelect(ch);
    delay(3);

    if (i2cDetect(TCS_ADDR)) {
      uint16_t r, g, b, c;
      tcs.getRawData(&r, &g, &b, &c);

      // store green channel for boundary logic
      analogValues[ch] = g;
    } else {
      analogValues[ch] = 0;
    }
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
  // assumes:
  // 0 = front
  // 2 = left
  // 4 = back
  // 6 = right

  if (analogValues[0] >= greenValues[0] + buffer) {
    return 180;
  }

  if (analogValues[2] >= greenValues[2] + buffer) {
    return 90;
  }

  if (analogValues[4] >= greenValues[4] + buffer) {
    return 0;
  }

  if (analogValues[6] >= greenValues[6] + buffer) {
    return 270;
  }

  return -1;
}

float ColorSensor::getWarningAvoidAngle() {
  return getAvoidAngle();
}