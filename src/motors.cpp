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
}

void WheelMotorDriver::update(motors_cmd_data_t& data_queue)
{
  setSpeed(data_queue * default_direction_);
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
  if (speed >= 0) {
    analogWrite(pwm_a_, 0);
    analogWrite(pwm_b_, speed);
  } else {
    analogWrite(pwm_a_, -speed);
    analogWrite(pwm_b_, 0);
  }
}

double WheelMotorDriver::angPoseUpdate()
{
  ang_pose_ = actual_encoder_value_ / TICK_PER_2PI_RAD * default_direction_;
  return ang_pose_;
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
