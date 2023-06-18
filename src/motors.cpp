#include "motors.h"

WheelMotorDriver::WheelMotorDriver(
  pin_size_t pwm_a_pin, pin_size_t pwm_b_pin, pin_size_t enc_a_pin, pin_size_t enc_b_pin,
  int8_t default_direction, uint8_t observer_id)
{
  pwm_a_ = pwm_a_pin;
  pwm_b_ = pwm_b_pin;
  enc_a_ = enc_a_pin;
  enc_b_ = enc_b_pin;

  default_direction_ = default_direction;
  unique_observers_id_ = observer_id;
}

void WheelMotorDriver::update(joint_states_data_t& data_queue)
{
  data_queue.actual_ang_pose = angPoseUpdate();
  data_queue.actual_ang_vel = angVelUpdate();
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

void WheelMotorDriver::setSpeed(int16_t speed)
{
  speed *= default_direction_;
  if (speed >= 0) {
    analogWrite(pwm_a_, speed);
    analogWrite(pwm_b_, 0);
  } else {
    analogWrite(pwm_a_, 0);
    analogWrite(pwm_b_, -speed);
  }
}

double WheelMotorDriver::angPoseUpdate()
{
  ang_pose_ = actual_encoder_value_ / TICK_PER_2PI_RAD * default_direction_;
  return ang_pose_;
}

double WheelMotorDriver::angVelUpdate()
{
  uint64_t time_now_us = time_us_64();
  double dt = (time_now_us - vel_last_time_us_);
  ang_vel_ = (actual_encoder_value_ - last_encoder_value_) / (TICK_PER_2PI_RAD * dt * 1e-6) *
             default_direction_;

  vel_last_time_us_ = time_now_us;
  last_encoder_value_ = actual_encoder_value_;

  return ang_vel_;
}
