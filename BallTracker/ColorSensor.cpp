#include <Wire.h>
#include "ColorSensor.h"

// ============================================================
//  TCA9548A mux channel select — activates one I2C branch
// ============================================================
void ColorSensor::tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire1.beginTransmission(TCA_ADDR);
  Wire1.write(1 << channel);
  Wire1.endTransmission();
}

// ============================================================
//  Quick I2C presence check
// ============================================================
bool ColorSensor::i2cDetect(uint8_t addr) {
  Wire1.beginTransmission(addr);
  return (Wire1.endTransmission() == 0);
}

// ============================================================
//  init() — Wire1 on pins 16/17, detect mux + all sensors,
//           then capture the green-field baseline.
// ============================================================
void ColorSensor::init() {
  // Wire1: SDA = pin 17, SCL = pin 16 (Teensy 4.1)
  Wire1.begin();

  for (int i = 0; i < NUM_SENSORS; i++) {
    analogValues[i] = 0;
    greenValues[i]  = 0;
  }

  if (!i2cDetect(TCA_ADDR)) {
    Serial.println("[ColorSensor] TCA9548A not found at 0x70 — check wiring!");
    return;
  }
  Serial.println("[ColorSensor] TCA9548A mux detected.");

  for (int i = 0; i < NUM_SENSORS; i++) {
    tcaSelect(i);
    if (!i2cDetect(TCS_ADDR)) {
      Serial.print("[ColorSensor] TCS34725 NOT found on CH");
      Serial.println(i);
    } else {
      if (!tcs.begin(TCS_ADDR, &Wire1)) {
        Serial.print("[ColorSensor] TCS34725 failed to start on CH");
        Serial.println(i);
      } else {
        Serial.print("[ColorSensor] TCS34725 OK on CH");
        Serial.println(i);
      }
    }
  }

  // Prime the readings array
  updateReadings();

  // Wait for front sensor (CH0) to give a non-zero value so baseline is valid
  int attempts = 0;
  while (analogValues[0] == 0 && attempts < 100) {
    updateReadings();
    delay(20);
    attempts++;
  }

  // Capture green-field baseline
  for (int i = 0; i < NUM_SENSORS; i++) {
    greenValues[i] = analogValues[i];
  }

  Serial.println("[ColorSensor] Baseline captured.");
}

// ============================================================
//  updateReadings() — poll every channel; store green channel
// ============================================================
void ColorSensor::updateReadings() {
  uint16_t r, g, b, c;
  for (int i = 0; i < NUM_SENSORS; i++) {
    tcaSelect(i);
    if (!i2cDetect(TCS_ADDR)) {
      analogValues[i] = 0;
      continue;
    }
    tcs.getRawData(&r, &g, &b, &c);
    analogValues[i] = g;   // green channel tracks field colour well
  }
}

// ============================================================
//  getAnalogValues() — raw green readings, all 8 channels
// ============================================================
uint16_t* ColorSensor::getAnalogValues() {
  return analogValues;
}

// ============================================================
//  printReadings() / printGreenValues() — debug helpers
// ============================================================
void ColorSensor::printReadings() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print("CH"); Serial.print(i);
    Serial.print(":"); Serial.print(analogValues[i]);
    Serial.print("  ");
  }
  Serial.println();
}

void ColorSensor::printGreenValues() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print("CH"); Serial.print(i);
    Serial.print(":"); Serial.print(greenValues[i]);
    Serial.print("  ");
  }
  Serial.println();
}

// ============================================================
//  getAvoidAngle()
//
//  Each sensor sits at a known angle around the robot (CW, 0=front).
//  When a sensor sees white (reading > baseline + buffer) we need
//  to move in the OPPOSITE direction — i.e. sensor_angle + 180°.
//
//  Channel → robot angle:
//    CH0 →   0° (front)      avoid → 180°
//    CH1 →  45° (front-right) avoid → 225°
//    CH2 →  90° (right)      avoid → 270°
//    CH3 → 135° (back-right)  avoid →  315°
//    CH4 → 180° (back)       avoid →   0°
//    CH5 → 225° (back-left)   avoid →  45°
//    CH6 → 270° (left)       avoid →  90°
//    CH7 → 315° (front-left)  avoid → 135°
//
//  Priority: front sensor first (most dangerous), then clockwise.
//  Returns -1.0 if no white detected.
// ============================================================
float ColorSensor::getAvoidAngle() {
  // Sensor angles in clockwise order (0° = front of robot)
  const float sensorAngle[NUM_SENSORS] = {
    0.0f, 45.0f, 90.0f, 135.0f, 180.0f, 225.0f, 270.0f, 315.0f
  };

  for (int i = 0; i < NUM_SENSORS; i++) {
    if (analogValues[i] >= (uint16_t)(greenValues[i] + buffer)) {
      float avoid = sensorAngle[i] + 180.0f;
      if (avoid >= 360.0f) avoid -= 360.0f;
      return avoid;
    }
  }
  return -1.0f;   // no white detected
}
