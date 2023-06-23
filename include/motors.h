#pragma once
#include <Arduino.h>
#include <time.h>

#include "uros/observers.h"

#define CMD_VEL_TIMEOUT_S 0.1
#define M1_DEFAULT_DIR -1  // -1 (CW) or 1 (CCW)
#define M2_DEFAULT_DIR 1

#define ENC_RESOLUTION 64
#define GEARBOX_RATIO 7.5
#define TICK_PER_2PI_RAD ((ENC_RESOLUTION * GEARBOX_RATIO) / (2 * PI))

class WheelMotorDriver : public EventObserverInterface<joint_states_data_t>,
                         public EventObserverInterface<motors_cmd_data_t>
{
public:
  WheelMotorDriver(
    pin_size_t pwm_a_pin, pin_size_t pwm_b_pin, pin_size_t enc_a_pin, pin_size_t enc_b_pin,
    int8_t default_direction, uint8_t observer_id);
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
  double ang_vel_enc_cnt_based_;
  double ang_vel_enc_dt_based_;
  double min_ang_vel_ = 0.1;
  double max_ang_vel = 35.0;

  const double kp_gain_ = 4.0;
  const double ki_gain_ = 2.5;
  const double kd_gain_ = 2.0;
  double last_error_ = 0.0;
  double error_sum_ = 0.0;
  int min_pwm_ = 7;

  unsigned long encoder_change_last_time_ = 0;
  unsigned long vel_last_time_ = 0;
  unsigned long cmd_update_last_time_;

  uint8_t unique_observers_id_;
};

class VacuumMotorDriver
{
public:
  VacuumMotorDriver() {}

private:
};