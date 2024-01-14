#pragma once
#include <Arduino.h>
#include <time.h>
#include <cmath>

#include "uros/observers.h"

// TODO: static constexpr
static const int8_t m1_default_dir = 1;  // -1 (CW) or 1 (CCW)
static const int8_t m2_default_dir = -1;

static const double cmd_vel_timeout = 0.1;
static const double min_ang_vel = 0.1;
static const double max_ang_vel = 35.0;
static const uint16_t min_pwm = 7;

static const uint16_t enc_resolution = 64;
static const double gearbox_ration = 15.0;
static const double tick_per_2pi_rad = ((enc_resolution * gearbox_ration) / (2 * PI));

static const double kp_gain = 4.0;
static const double ki_gain = 1.5;
static const double kd_gain = 2.0;
static const double vel_filter_coeff_a = 0.7767416;
static const double vel_filter_coeff_b = 0.1116292;

class WheelMotorDriver : public EventObserverInterface<joint_states_data_t>,
                         public EventObserverInterface<motors_cmd_data_t>
{
public:
  WheelMotorDriver(
    pin_size_t pwm_a_pin, pin_size_t pwm_b_pin, pin_size_t enc_a_pin, pin_size_t enc_b_pin,
    int8_t default_direction);
  ~WheelMotorDriver() {}

  void readEncoder();

protected:
  void update(joint_states_data_t& data_queue) override;
  void update(motors_cmd_data_t& data_queue) override;

private:
  void pidControlLoop();
  double angVelUpdate();
  double angPoseUpdate();
  void setSpeed(int16_t speed);

  pin_size_t pwm_a_;
  pin_size_t pwm_b_;
  pin_size_t enc_a_;
  pin_size_t enc_b_;
  int8_t default_direction_;

  int64_t actual_encoder_value_ = 0;
  int64_t last_encoder_value_ = 0;

  double set_point_;
  double ang_pose_;
  double ang_vel_;
  double ang_vel_filtered_;
  double ang_vel_enc_cnt_based_;
  double ang_vel_enc_dt_based_;
  double ang_vel_prev_ = 0.0;

  double last_error_ = 0.0;
  double error_sum_ = 0.0;

  unsigned long encoder_change_last_time_ = 0;
  unsigned long vel_last_time_ = 0;
  unsigned long cmd_update_last_time_;
};

class VacuumMotorDriver
{
public:
  VacuumMotorDriver() {}

private:
};