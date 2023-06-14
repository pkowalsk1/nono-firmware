#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include "CytronMotorDriver.h"

class WheelMotorDriver
{
public:
  WheelMotorDriver(
    pin_size_t pwm_a_pin, pin_size_t pwm_b_pin, pin_size_t enc_a_pin, pin_size_t enc_b_pin);

  void readEncoder();

private:
  int64_t enc_counter_ = 0;

  pin_size_t pwm_a_;
  pin_size_t pwm_b_;
  pin_size_t enc_a_;
  pin_size_t enc_b_;
};

class VacuumMotorDriver
{
public:
  VacuumMotorDriver() {}

private:
};

#endif /* MOTORS_H */