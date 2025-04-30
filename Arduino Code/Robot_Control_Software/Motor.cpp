#include <Arduino.h>
#include <stdint.h>
#include "Setup.h"
#include "Motor.h"

Motor::Motor(uint8_t pwm, uint8_t plus, uint8_t minus) : pwmPin(pwm), plusPin(plus), minusPin(minus) {}

void Motor::rotateCW(uint8_t pwm) {
  digitalWrite(plusPin, HIGH);
  digitalWrite(minusPin, LOW);
  digitalWrite(pwmPin, pwm);
}
void Motor::rotateCCW(uint8_t pwm) {
  digitalWrite(plusPin, LOW);
  digitalWrite(minusPin, HIGH);
  digitalWrite(pwmPin, pwm);
}

void Motor::stop() {
  digitalWrite(pwmPin, 0);
}