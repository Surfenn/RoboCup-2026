#include "motor.h"         // Includes the Motor class declarations from motor.h
#include <Arduino.h>       // Includes Arduino functions like pinMode() and analogWrite()

// Constructor for the Motor class
Motor::Motor(unsigned int pin_c, unsigned int pin_cc, uint16_t pwmMax)
    : pin_c(pin_c),        // Initializes the clockwise/control pin member variable
      pin_cc(pin_cc),      // Initializes the counterclockwise/reverse pin member variable
      pwm_max(pwmMax),     // Initializes the maximum allowed PWM value
      last_c(0),           // Stores the last PWM sent to pin_c, starts at 0
      last_cc(0)           // Stores the last PWM sent to pin_cc, starts at 0
{
    pinMode(pin_c, OUTPUT);    // Sets pin_c as an output pin
    pinMode(pin_cc, OUTPUT);   // Sets pin_cc as an output pin
}

// Spins the motor at speed s
void Motor::spin(int s){
    // Limit to safe range [-pwm_max, pwm_max]

    if (s > (int)pwm_max){         // If requested speed is above max allowed
        s = (int)pwm_max;          // Clamp it down to pwm_max
    } 

    if (s < -(int)pwm_max){        // If requested speed is below negative max allowed
        s = -(int)pwm_max;         // Clamp it up to -pwm_max
    } 

    uint16_t out_c = 0;            // PWM output for forward/clockwise direction, starts at 0
    uint16_t out_cc = 0;           // PWM output for reverse/counterclockwise direction, starts at 0

    if (s >= 0){                   // If speed is positive or zero
        out_c = (uint16_t)s;       // Send PWM to forward pin
        // analogWrite(pin_c, s);  // Old direct write approach, now replaced by optimized logic
        // analogWrite(pin_cc, 0); // Would force reverse pin off
    } 
    else if (s < 0){               // If speed is negative
        out_cc = (uint16_t)(-s);   // Convert negative speed to positive PWM for reverse pin
        // analogWrite(pin_cc, -s); // Old direct write approach, now replaced by optimized logic
        // analogWrite(pin_c, 0);   // Would force forward pin off
    }

    // Only update PWM when it actually changes
    if (out_c == last_c && out_cc == last_cc){   // If outputs are exactly the same as last time
        return;                                  // Do nothing to avoid unnecessary writes
    }
    
    last_c = out_c;              // Save new forward PWM as the last written value
    last_cc = out_cc;            // Save new reverse PWM as the last written value

    analogWrite(pin_c, out_c);   // Write PWM to forward pin
    analogWrite(pin_cc, out_cc); // Write PWM to reverse pin
}

// Stops the motor by writing 0 to both control pins
void Motor::brake()
{
    if (last_c == 0 && last_cc == 0){   // If motor is already stopped
        return;                         // Do nothing
    }

    last_c = 0;                         // Update stored forward PWM to 0
    last_cc = 0;                        // Update stored reverse PWM to 0

    analogWrite(pin_c, 0);              // Turn off forward PWM
    analogWrite(pin_cc, 0);             // Turn off reverse PWM
}