#include "motors.h"

WheelMotorDriver::WheelMotorDriver(
  pin_size_t pwm_a_pin, pin_size_t pwm_b_pin, pin_size_t enc_a_pin, pin_size_t enc_b_pin,
  int8_t default_direction, uint8_t observer_id)
: pwm_a_(pwm_a_pin),
  pwm_b_(pwm_b_pin),
  enc_a_(enc_a_pin),
  enc_b_(enc_b_pin),
  default_direction_(default_direction),
  unique_observers_id_(observer_id)
{}

void WheelMotorDriver::update(joint_states_data_t& data_queue)
{
  data_queue.actual_ang_pose = angPoseUpdate();
  data_queue.actual_ang_vel = angVelUpdate();

  pidControlLoop();
}

void WheelMotorDriver::update(motors_cmd_data_t& data_queue)
{
  set_point_ = data_queue;
  last_cmd_update_time_ = micros();
}

void WheelMotorDriver::pidControlLoop()
{
  if (micros() - last_cmd_update_time_ >= 0.1 * 1e6) {
    set_point_ = 0;
  } 
  // else {
  //   set_point_ *= (double)default_direction_;
  // }

  unsigned long current_time = micros();
  unsigned long elapsed_time = current_time - pid_last_time_;

  actual_error_ = (ang_vel_ - set_point_) * (double)default_direction_;

  double p_term = kp_gain_ * actual_error_;
  double i_term = ki_gain_ * (error_sum_ + actual_error_);
  double d_term = kd_gain_ * (actual_error_ - last_error_);

  pid_out_ = p_term + i_term + d_term;
  pid_out_ = constrain(pid_out_, -255, 255);

  setSpeed(pid_out_);

  last_error_ = actual_error_;
  error_sum_ += actual_error_;
  pid_last_time_ = current_time;
}

double WheelMotorDriver::angVelUpdate()
{
  long time_now_us = micros();
  float dt = ((float)(time_now_us - vel_last_time_us_)) / (1.0e6);
  ang_vel_ =
    (actual_encoder_value_ - last_encoder_value_) / (TICK_PER_2PI_RAD * dt) * default_direction_;

  vel_last_time_us_ = time_now_us;
  last_encoder_value_ = actual_encoder_value_;

  return ang_vel_;
}

double WheelMotorDriver::angPoseUpdate()
{
  ang_pose_ = actual_encoder_value_ / TICK_PER_2PI_RAD * default_direction_;
  return ang_pose_;
}

void WheelMotorDriver::setSpeed(int16_t speed)
{
  speed = constrain(speed, -255, 255);

  if (speed >= 0) {
    analogWrite(pwm_a_, speed);
    analogWrite(pwm_b_, 0);
  } else {
    analogWrite(pwm_a_, 0);
    analogWrite(pwm_b_, -speed);
  }
}

void WheelMotorDriver::readEncoder()
{
  int b = digitalRead(enc_b_);

  if (b > 0) {
    actual_encoder_value_++;
  } else {
    actual_encoder_value_--;
  }
}