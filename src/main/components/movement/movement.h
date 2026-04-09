#include "WString.h"
#ifndef MOVEMENT_H
#define MOVEMENT_H

#include "../motor/motor.cpp"
#include "../compass/compass.cpp"

class Movement {

public:
  void initMovement();
  void debug();
  void brake();
  void move(double theta, int maxSpeed, bool avoid, float cameraRotationAngle);
  void rotate(int speed);
  void debug_sees_border();
  bool is_on_border();
  void rotate_motor(int speed, String motor);
  void basic_move_with_compass(double theta, int maxSpeed);
  void basic_move_with_compass_and_camera(double theta, int maxSpeed, float camAngle);

private:
  Compass compass;

  // Robot 1 White
  Motor motor_FR{ 1, 2 }; //M1
  Motor motor_BR{ 4, 3 }; //M2
  Motor motor_BL{ 5, 6 }; //M3
  Motor motor_FL{ 7, 8 }; //M4



  bool isBetween(int lower, int upper, int x);
  const unsigned int COMPASS_BUFF = 5;
  float spin_index = 0;
};

#endif