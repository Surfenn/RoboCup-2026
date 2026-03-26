#include <Arduino.h>
#include "./components/IR/IR.h"
#include "./components/IR/IR.cpp"
#include "./components/colorsensor/colorsensor.h"
#include "./components/colorsensor/colorsensor.cpp"
#include "./components/movement/movement.h"
#include "./components/movement/movement.cpp"


Movement m;
IR ir;
ColorSensor c;
// Camera camera(70.0);

void attack_w_color_sensor() {
  int speed = 80;

  c.updateReadings();
  ir.updateReadings();

  float curr_ball_angle = ir.getBallAngle();
  float avoidAngle = c.getAvoidAngle();

  Serial.println("green:");
  c.printGreenValues();
  Serial.println("readings:");
  c.printReadings();

  if (avoidAngle != -1) {
    m.basic_move_with_compass(avoidAngle, 175);
    delay(200);
    Serial.println("oob");
  } else {
    m.basic_move_with_compass(curr_ball_angle, speed);
  }
}

void setup() {
  Serial.begin(9600);
  ir.initIR();
  m.initMovement();
  c.init();

  // camera.initialize();
}

void loop() {
  attack_w_color_sensor();
  ir.printReadingsArr();
  delay(10);
}