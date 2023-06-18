#pragma once
#include <Arduino.h>
#include <time.h>

// #include "uros/observers.h"

#define M1_DEFAULT_DIR 1  // -1 (CW) or 1 (CCW)
#define M2_DEFAULT_DIR -1

#define ENC_RESOLUTION 64
#define GEARBOX_RATIO 7.5
#define TICK_PER_2PI_RAD ((ENC_RESOLUTION * GEARBOX_RATIO) / (2 * PI))

class WheelMotorDriver  // : public EventObserverInterface<odom_data_t>
{
public:
  WheelMotorDriver(
    pin_size_t pwm_a_pin, pin_size_t pwm_b_pin, pin_size_t enc_a_pin, pin_size_t enc_b_pin,
    int8_t default_direction);

  void readEncoder();
  void setSpeed(int16_t speed);
  double angPoseUpdate();
  double angVelUpdate();

private:
  pin_size_t pwm_a_;
  pin_size_t pwm_b_;
  pin_size_t enc_a_;
  pin_size_t enc_b_;
  int8_t default_direction_;

  int64_t actual_encoder_value_ = 0;
  int64_t last_encoder_value_ = 0;
  int64_t vel_last_time_us_ = 0;

  double ang_pose_;
  double ang_vel_;
};

class VacuumMotorDriver
{
public:
  VacuumMotorDriver() {}

private:
};