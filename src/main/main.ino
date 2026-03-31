#include "./components/IR/IR.h"
#include "./components/IR/IR.cpp"
#include "./components/movement/movement.h"
#include "./components/movement/movement.cpp"

Movement m;
IR ir;
Compass cmp;

void attack_ball() {
  int speed = 120;

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

// void setup() {  
//   Serial.begin(9600);

//   ir.initIR();
//   m.initMovement();
//   cmp.initialize();
// }

void setup() {  //for testing
  Serial.begin(9600);

  ir.initIR();
  m.initMovement();
}

void loop() {
  attack_ball();
}