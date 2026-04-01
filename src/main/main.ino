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
  int speed = 120;

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