#include "RobotMotors.h"
#include <Arduino.h>
#include <math.h>

RobotMotors::RobotMotors() {}

void RobotMotors::begin() {
  pinMode(motor_FR_PWM, OUTPUT);
  pinMode(motor_FR_Dir, OUTPUT);
  pinMode(motor_BR_PWM, OUTPUT);
  pinMode(motor_BR_Dir, OUTPUT);
  pinMode(motor_BL_PWM, OUTPUT);
  pinMode(motor_BL_Dir, OUTPUT);
  pinMode(motor_FL_PWM, OUTPUT);
  pinMode(motor_FL_Dir, OUTPUT);
  brake();
}

/**
 * Commands the 4-wheel omni-drive robot.
 * angle: Direction to travel in degrees (0=Forward, 90=Right, 180=Back,
 * 270=Left) [CW] speed: Translation speed (0-255) turn: Rotational speed (-255
 * to 255, positive = Clockwise rotation)
 */
void RobotMotors::drive(float angle, float speed, float turn) {
  if (speed < 5 && abs(turn) < 5) {
    brake();
    return;
  }

  // Empirically verified: the wheel matrix is 90° offset from our CW system.
  // Motor test showed: N→W, E→N, W→S (all shifted -90°).
  // Fix: use 270 - angle instead of 360 - angle.
  float angle_ccw = 270.0f - angle;
  if (angle_ccw < 0.0f)
    angle_ccw += 360.0f;

  float rad = angle_ccw * PI / 180.0;

  // Wheel matrix using 40-degree offsets from the lateral axis (90 deg).
  // These provide the translational components for an X-drive.
  float fr_t = speed * sin(rad + (130.0 * PI / 180.0));
  float br_t = speed * sin(rad + (50.0 * PI / 180.0));
  float bl_t = speed * sin(rad - (50.0 * PI / 180.0));
  float fl_t = speed * sin(rad - (130.0 * PI / 180.0));

  // Mix translation and rotation.
  // 'turn' is positive for Clockwise rotation.
  // Since turn is added to all motors, and BL/FL are physically inverted
  // in their direction pins, this creates the necessary differential.
  float fr = fr_t + turn;
  float br = br_t + turn;
  float bl = bl_t + turn;
  float fl = fl_t + turn;

  // Constrain to PWM range
  float maxVal = abs(fr);
  if (abs(br) > maxVal)
    maxVal = abs(br);
  if (abs(bl) > maxVal)
    maxVal = abs(bl);
  if (abs(fl) > maxVal)
    maxVal = abs(fl);

  if (maxVal > 255) {
    float scale = 255.0 / maxVal;
    fr *= scale;
    br *= scale;
    bl *= scale;
    fl *= scale;
  }

  // Motor FR
  analogWrite(motor_FR_PWM, (int)abs(fr));
  digitalWrite(motor_FR_Dir, fr >= 0 ? HIGH : LOW);

  // Motor BR
  analogWrite(motor_BR_PWM, (int)abs(br));
  digitalWrite(motor_BR_Dir, br >= 0 ? HIGH : LOW);

  // Motor BL (Inverted direction logic: LOW is positive)
  analogWrite(motor_BL_PWM, (int)abs(bl));
  digitalWrite(motor_BL_Dir, bl >= 0 ? LOW : HIGH);

  // Motor FL (Inverted direction logic: LOW is positive)
  analogWrite(motor_FL_PWM, (int)abs(fl));
  digitalWrite(motor_FL_Dir, fl >= 0 ? LOW : HIGH);
}

void RobotMotors::brake() {
  analogWrite(motor_FR_PWM, 0);
  analogWrite(motor_BR_PWM, 0);
  analogWrite(motor_BL_PWM, 0);
  analogWrite(motor_FL_PWM, 0);
}
