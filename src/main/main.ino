#include "./components/IR/IR.h"
#include "./components/IR/IR.cpp"
#include "./components/movement/movement.h"
#include "./components/movement/movement.cpp"

Movement m;
IR ir;
Compass cmp;

void attack_ball() {
  int speed = 120;  // adjust if needed

  ir.updateReadings();
  float ballAngle = ir.getBallAngle();

  Serial.print("Ball angle: ");
  Serial.println(ballAngle);

  // If your IR returns -1 when no ball is found
  if (ballAngle == -1) {
    m.brake();
    Serial.println("No ball detected");
    return;
  }

  // Move toward the ball
  m.basic_move_with_compass(ballAngle, speed);
}

void setup() {  
  Serial.begin(9600);

  ir.initIR();
  m.initMovement();
  cmp.initialize();
}

void loop() {
  attack_ball();
}