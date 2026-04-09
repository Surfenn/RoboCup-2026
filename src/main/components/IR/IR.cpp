#include "IR.h"
#include <limits.h>

int pins[] = {23, 24, 25, 26, 27, 38, 39, 40, 41, 14, 15, 18, 19, 20, 21, 22}; //{23, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 16, 17, 20, 21, 22};
const int NUM_IR_PINS = 16;

double readings[NUM_IR_PINS];

void IR::initIR() {
  for (unsigned int i = 0; i < arrayLength(pins); i++) {
    pinMode(pins[i], INPUT_PULLUP);
  }
}

void IR::updateReadings() {
  double pwReadings[NUM_IR_PINS];  // stack buffer, read once
  double minVal = INT_MAX;
  double maxVal = INT_MIN;

  for (int i = 0; i < NUM_IR_PINS; i++) {
    unsigned long rawPulse = pulseIn(pins[i], HIGH, 2500);
    
    if (rawPulse == 0) {
      if (digitalRead(pins[i]) == LOW) {
        // Debounce: Wait 200us and double-check to mathematically guarantee it's not microscopic I2C/UART data line noise!
        delayMicroseconds(200);
        if (digitalRead(pins[i]) == LOW) {
          pwReadings[i] = 1; // Genuine long-duration IR saturation holding LOW -> Strongest possible signal!
        } else {
          pwReadings[i] = 0; // Was just a high-speed data blip, safely ignore
        }
      } else {
        pwReadings[i] = 0; // Truly zero signal
      }
    } else {
      pwReadings[i] = rawPulse;
    }

    if (pwReadings[i] != 0 && pwReadings[i] < minVal) minVal = pwReadings[i];
    if (pwReadings[i] > maxVal) maxVal = pwReadings[i];
  }

  if (minVal == INT_MAX || maxVal == INT_MIN) {
    for (int i = 0; i < NUM_IR_PINS; i++) readings[i] = 0;
    return;
  }

  double range = maxVal - minVal;
  if (range == 0.0) range = 1.0; // avoid division by zero
  for (int i = 0; i < NUM_IR_PINS; i++) {
    readings[i] = (pwReadings[i] == 0) ? 0 : (1.0 - ((pwReadings[i] - minVal) / range));
  }
}

float* IR::getReadingsArr() {
  // Use static buffers instead of heap allocation.
  // new[] on every loop() call exhausts SRAM in milliseconds — never use
  // heap allocation inside a hot path on a microcontroller.
  static double pwReadings[NUM_IR_PINS];
  static float  pinReadings[NUM_IR_PINS];
 
  // Fill pwReadings in-place (no allocation)
  for (unsigned int i = 0; i < arrayLength(pins); i++) {
    pwReadings[i] = pulseIn(pins[i], HIGH, 800);
  }
 
  double minVal = INT_MAX;
  double maxVal = INT_MIN;
 
  for (int i = 0; i < NUM_IR_PINS; i++) {
    if (pwReadings[i] != 0 && pwReadings[i] < minVal) minVal = pwReadings[i];
    if (pwReadings[i] > maxVal) maxVal = pwReadings[i];
  }
 
  if (minVal == INT_MAX || maxVal == INT_MIN) {
    for (int i = 0; i < NUM_IR_PINS; i++) {
      pinReadings[i] = 0;
    }
    return pinReadings;
  }
 
  double range = maxVal - minVal;
  if (range == 0.0) range = 1.0;
  for (int i = 0; i < NUM_IR_PINS; i++) {
    if (pwReadings[i] == 0) {
      pinReadings[i] = 0;
    }
    else {
      pinReadings[i] = (float)(1 - ((pwReadings[i] - minVal) / range));
    }
  }
 
  return pinReadings;
}

float IR::getBallAngle() {
  int n = NUM_IR_PINS;
  float sharpen = 2.5;
  float weightedX = 0.0;
  float weightedY = 0.0;
  float totalWeight = 0.0;

  for (int i = 0; i < n; i++) {
    float weight = readings[i];

    if (weight <= 0.0) continue;

    float angleRad = (-i * 2.0 * M_PI) / n;
    float adjustedWeight = pow(weight, sharpen);

    weightedX += adjustedWeight * cos(angleRad);
    weightedY += adjustedWeight * sin(angleRad);
    totalWeight += adjustedWeight;
  }

  if (totalWeight == 0.0) {
    return -1; // No ball detected
  }

  // Smooth the raw Cartesian vectors (this natively handles 360-degree wrap-around safely)
  float alpha = 0.35; // Lower is smoother but slower to adapt. 0.35 is a responsive median.
  avgX = (alpha * weightedX) + ((1.0 - alpha) * avgX);
  avgY = (alpha * weightedY) + ((1.0 - alpha) * avgY);

  float angle = atan2(avgY, avgX) * 180.0 / M_PI;
  if (angle < 0) angle += 360.0;

  // angle += 30;   // adjust this value as needed
  if (angle >= 360.0) angle -= 360.0;

  return angle;
}

double* IR::getPWsArr() {
  static double pinReadings[NUM_IR_PINS];
  for (unsigned int i = 0; i < arrayLength(pins); i++) {
    pinReadings[i] = pulseIn(pins[i], HIGH, 800);
  }
  return pinReadings;
}

// void IR::printReadingsArr() {
//   Serial.print("[ ");
//   for(int i = 0; i < NUM_IR_PINS; i++) {
//     Serial.print(readings[i]);
//     Serial.print(" ");
//   }
//   Serial.println("]");
// }

// void IR::printPWsArr() {
//   double* arr = getPWsArr();
//   Serial.print("[ ");
//   for(int i = 0; i < NUM_IR_PINS; i++) {
//     Serial.print(arr[i]);
//     Serial.print(" ");
//   }
//   Serial.println("]");
// }