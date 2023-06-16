#pragma once
#include <Arduino.h>

// #include "uros/observers.h"

class WheelMotorDriver  // : public EventObserverInterface<odom_data_t>
{
public:
  WheelMotorDriver(
    pin_size_t pwm_a_pin, pin_size_t pwm_b_pin, pin_size_t enc_a_pin, pin_size_t enc_b_pin);

  int64_t getEncPos() const
  {
    return enc_counter_;
  };

  void readEncoder();
  void setSpeed(int16_t speed);

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