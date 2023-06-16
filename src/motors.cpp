#include "motors.h"

WheelMotorDriver::WheelMotorDriver(
  pin_size_t pwm_a_pin, pin_size_t pwm_b_pin, pin_size_t enc_a_pin, pin_size_t enc_b_pin)
{
  pwm_a_ = pwm_a_pin;
  pwm_b_ = pwm_b_pin;
  enc_a_ = enc_a_pin;
  enc_b_ = enc_b_pin;
}

void WheelMotorDriver::readEncoder()
{
  int b = digitalRead(enc_b_);

  if (b > 0) {
    enc_counter_++;
  } else {
    enc_counter_--;
  }
}

void WheelMotorDriver::setSpeed(int16_t speed)
{
  if (speed >= 0) {
    analogWrite(pwm_a_, speed);
    analogWrite(pwm_b_, 0);
  } else {
    analogWrite(pwm_a_, 0);
    analogWrite(pwm_b_, -speed);
  }
}