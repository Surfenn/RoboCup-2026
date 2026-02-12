#include "motor.h"

Motor::Motor(unsigned int pin_c, unsigned int pin_cc, uint16_t pwmMax)
    : pin_c(pin_c), 
      pin_cc(pin_cc), 
      pwm_max(pwmMax), 
      last_c(0), 
      last_cc(0)
{
    pinMode(pin_c, OUTPUT);
    pinMode(pin_cc, OUTPUT);
}

void Motor::spin(int s){
    // Limit to safe range [-pwm_max, pwm_max]
    if (s > (int)pwm_max){
        s = (int)pwm_max;
    } 
    if (s < -(int)pwm_max){
        s = -(int)pwm_max;
    } 

    uint16_t out_c = 0;
    uint16_t out_cc = 0;

    if (s >= 0){
        out_c = (uint16_t)s;
        // analogWrite(pin_c, s);
        // analogWrite(pin_cc, 0);
    } 
    else if (s < 0){
        out_cc = (uint16_t)(-s);
        // analogWrite(pin_cc, -s);
        // analogWrite(pin_c, 0);
    }

    // Only update PWM when it actually changes
    if (out_c == last_c && out_cc == last_cc){
        return;
    }
    
    last_c = out_c;
    last_cc = out_cc;

    analogWrite(pin_c, out_c);
    analogWrite(pin_cc, out_cc);
    


}

void Motor::brake()
{
    if (last_c == 0 && last_cc == 0){
        return;
    }

    last_c = 0;
    last_cc = 0;

    analogWrite(pin_c, 0);
    analogWrite(pin_cc, 0);

}