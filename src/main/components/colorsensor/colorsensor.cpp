#include "colorsensor.h"
#include <Wire.h>
#include "Adafruit_TCS34725.h"

// I2C addresses
static const uint8_t TCA_ADDR = 0x70;
static const uint8_t TCS_ADDR = 0x29;

// Threshold above baseline to detect boundary
static const uint16_t BUFFER = 150;

// Arrays (8 sensors)
static uint16_t analogValues[ColorSensor::NUM_SENSORS];
static uint16_t greenValues[ColorSensor::NUM_SENSORS];

// Re-used sensor object (we switch mux channels before reading)
static Adafruit_TCS34725 tcs(
  TCS34725_INTEGRATIONTIME_50MS,
  TCS34725_GAIN_4X
);

// ---------- helpers ----------
static void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire1.beginTransmission(TCA_ADDR);
  Wire1.write(1 << channel);
  Wire1.endTransmission();
}

static bool i2cDetect(uint8_t addr) {
  Wire1.beginTransmission(addr);
  return (Wire1.endTransmission() == 0);
}

// Choose what value you want to treat as “analog” for line detect.
// Green works if line is green-ish; Clear often works best for white lines.
static uint16_t readSensorValue(uint8_t ch) {
  tcaSelect(ch);
  delayMicroseconds(500);

  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);

  return g;   // use GREEN (switch to `return c;` if your line is white)
  // return c;
}

// ---------- class methods ----------
void ColorSensor::init() {
  Wire1.begin();

  // Detect mux
  if (!i2cDetect(TCA_ADDR)) {
    Serial.println("ERROR: TCA9548A (0x70) not found on Wire1.");
    while (1) {}
  }

  // Detect & init each sensor on channels 0..7
  for (uint8_t ch = 0; ch < NUM_SENSORS; ch++) {
    tcaSelect(ch);
    delay(5);

    if (!i2cDetect(TCS_ADDR)) {
      Serial.print("ERROR: TCS34725 (0x29) not found on mux channel ");
      Serial.println(ch);
      while (1) {}
    }

    if (!tcs.begin(TCS_ADDR, &Wire1)) {
      Serial.print("ERROR: TCS34725 begin() failed on channel ");
      Serial.println(ch);
      while (1) {}
    }
  }

  // Take baseline (assumes robot starts on field, not on line)
  delay(200);
  updateReadings();
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    greenValues[i] = analogValues[i];
  }
}

uint16_t* ColorSensor::getAnalogValues() {
  return analogValues;
}

void ColorSensor::updateReadings() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    analogValues[i] = readSensorValue(i);
  }
}

void ColorSensor::printReadings() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    Serial.print(i);
    Serial.print(": ");
    Serial.print(analogValues[i]);
    Serial.print("\t");
  }
  Serial.println();
}

void ColorSensor::printGreenValues() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    Serial.print(i);
    Serial.print(": ");
    Serial.print(greenValues[i]);
    Serial.print("\t");
  }
  Serial.println();
}

float ColorSensor::getAvoidAngle() {
  float weightedX = 0.0f;
  float weightedY = 0.0f;
  float totalWeight = 0.0f;

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    int diff = (int)analogValues[i] - (int)greenValues[i];
    if (diff <= (int)BUFFER) continue;

    float w = (float)(diff - (int)BUFFER);
    float angleRad = (i * 2.0f * (float)M_PI) / (float)NUM_SENSORS;

    weightedX += w * cosf(angleRad);
    weightedY += w * sinf(angleRad);
    totalWeight += w;
  }

  if (totalWeight <= 0.0f) return -1.0f;

  float lineAngle = atan2f(weightedY, weightedX) * (180.0f / (float)M_PI);
  if (lineAngle < 0) lineAngle += 360.0f;

  // Move away from line direction
  return fmodf(lineAngle + 180.0f, 360.0f);
}