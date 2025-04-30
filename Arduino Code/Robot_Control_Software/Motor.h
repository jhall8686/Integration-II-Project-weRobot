#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <stdint.h>
//Motor Class

class Motor {
  private:
    uint8_t pwmPin;
    uint8_t plusPin;
    uint8_t minusPin;
  public:
    Motor(uint8_t pwm, uint8_t plus, uint8_t minus);
    void rotateCW(uint8_t pwm);
    void rotateCCW(uint8_t pwm);
    void stop();

}

#endif
