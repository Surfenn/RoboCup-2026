#include "./components/IR/IR.h"
#include "./components/IR/IR.cpp"
#include "./components/movement/movement.h"
#include "./components/movement/movement.cpp"
#include "./components/colorsensor/colorsensor.h"
#include "./components/colorsensor/colorsensor.cpp"

Movement m;
IR ir;
ColorSensor c;
Compass cmp;

void attack_ball() {
  int speed = 40;

  // First update color sensors so white border always has priority
  c.updateReadings();
  float avoidAngle = c.getAvoidAngle();

  // If white border is detected, do NOT chase the ball
  if (avoidAngle != -1) {
    Serial.print("White detected! Avoid angle: ");
    Serial.println(avoidAngle);

    m.basic_move_with_compass(avoidAngle, speed);
    return;
  }

  // Otherwise do the normal IR ball chasing
  ir.updateReadings();
  float ballAngle = ir.getBallAngle();

  float* vals = ir.getReadingsArr();
  for (int i = 0; i < NUM_IR_PINS; i++) {
    Serial.print(i);
    Serial.print(":");
    Serial.print(vals[i], 2);
    Serial.print("  ");
  }
  Serial.println();

  Serial.print("Ball angle: ");
  Serial.println(ballAngle);

  if (ballAngle == -1) {
    m.brake();
    Serial.println("No ball detected");
    return;
  }

  m.basic_move_with_compass(ballAngle, speed);
}

void setup() {
  Serial.begin(9600);

  ir.initIR();
  m.initMovement();
  c.init();

  // Add this back only if your movement code needs compass initialized
  // cmp.initialize();
}

void loop() {
  attack_ball();
}

// #include <Wire.h>
// #include "Adafruit_TCS34725.h"

// #define TCA_ADDR 0x70
// #define TCS_ADDR 0x29
// #define NUM_SENSORS 8

// Adafruit_TCS34725 tcs = Adafruit_TCS34725(
//   TCS34725_INTEGRATIONTIME_50MS,
//   TCS34725_GAIN_4X
// );

// uint16_t greenValues[NUM_SENSORS];
// uint16_t clearValues[NUM_SENSORS];
// uint16_t redValues[NUM_SENSORS];
// uint16_t blueValues[NUM_SENSORS];

// void tcaSelect(uint8_t channel) {
//   if (channel > 7) return;

//   Wire1.beginTransmission(TCA_ADDR);
//   Wire1.write(1 << channel);
//   Wire1.endTransmission();
// }

// bool i2cDetect(uint8_t addr) {
//   Wire1.beginTransmission(addr);
//   return (Wire1.endTransmission() == 0);
// }

// void setup() {
//   Serial.begin(9600);
//   while (!Serial) {}

//   Wire1.begin();

//   Serial.println("=== COLOR SENSOR CHANNEL TEST ===");
//   Serial.println("This will print CH0 to CH7.");
//   Serial.println("Move a white object over one sensor at a time.");
//   Serial.println("Watch which channel changes most.");
//   Serial.println();

//   // Check multiplexer
//   Serial.print("Checking TCA9548A at 0x70... ");
//   if (i2cDetect(TCA_ADDR)) {
//     Serial.println("FOUND");
//   } else {
//     Serial.println("NOT FOUND");
//     while (1) {}
//   }

//   // Check each channel for a TCS3472
//   for (int i = 0; i < NUM_SENSORS; i++) {
//     tcaSelect(i);

//     Serial.print("Checking TCS3472 on CH");
//     Serial.print(i);
//     Serial.print("... ");

//     if (i2cDetect(TCS_ADDR)) {
//       Serial.println("FOUND");

//       if (!tcs.begin(TCS_ADDR, &Wire1)) {
//         Serial.print("BUT tcs.begin() FAILED on CH");
//         Serial.println(i);
//       }
//     } else {
//       Serial.println("NOT FOUND");
//     }
//   }

//   Serial.println();
//   Serial.println("Start testing...");
//   Serial.println();
// }

// void loop() {
//   uint16_t r, g, b, c;

//   for (int i = 0; i < NUM_SENSORS; i++) {
//     tcaSelect(i);

//     if (!i2cDetect(TCS_ADDR)) {
//       redValues[i] = 0;
//       greenValues[i] = 0;
//       blueValues[i] = 0;
//       clearValues[i] = 0;
//       continue;
//     }

//     tcs.getRawData(&r, &g, &b, &c);

//     redValues[i] = r;
//     greenValues[i] = g;
//     blueValues[i] = b;
//     clearValues[i] = c;
//   }

//   Serial.println("--------------------------------------------------");
//   for (int i = 0; i < NUM_SENSORS; i++) {
//     Serial.print("CH");
//     Serial.print(i);
//     Serial.print("   R: ");
//     Serial.print(redValues[i]);
//     Serial.print("   G: ");
//     Serial.print(greenValues[i]);
//     Serial.print("   B: ");
//     Serial.print(blueValues[i]);
//     Serial.print("   C: ");
//     Serial.println(clearValues[i]);
//   }

//   Serial.println();
//   delay(300);
// }