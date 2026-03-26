#ifndef MOTOR_H                 // Include guard start to prevent this header from being included multiple times
#define MOTOR_H

// Motor class controls one motor using two output pins
class Motor
{
public:
  /**
   * Initializes the motor pins to be used.
   *
   * @param `pin_c` the pin that, when held high and the other held low, roatates the motor clockwise.
   * @param `pin_cc` the pin that, when held high and the other held low, roatates the motor counter-clockwise.
   */
  Motor(unsigned int pin_c, unsigned int pin_cc, uint16_t pwmMax = 255);  // Constructor: sets control pins and optional max PWM value

  /**
   * Spins the wheel at a certain speed. If the integer for speed that is given is > 0, the wheel will spin clockwise. If the opposite, the wheel will spin counter-clockwise.
   */
  void spin(int speed);         // Spins the motor forward or backward depending on the sign of speed

  /**
   * Stops the spin of the wheel; brakes it.
   */
  void brake();                 // Stops the motor by setting both outputs to 0

private:
  unsigned int pin_c;           // Stores the clockwise/forward control pin number
  unsigned int pin_cc;          // Stores the counter-clockwise/reverse control pin number

  uint16_t pwm_max;             // Stores the maximum allowed PWM output value

  uint16_t last_c;              // Stores the last PWM value sent to pin_c
  uint16_t last_cc;             // Stores the last PWM value sent to pin_cc
  // int speed;                 // Old unused variable, currently commented out
  // bool is_breaking;          // Old unused variable, currently commented out
};

#endif                          // End of include guard