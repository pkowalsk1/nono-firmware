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



// // ROS

// /* motors */
// motor_state_msg.position.data = motor_position;
// RCSOFTCHECK(rcl_publish(&motor_state_publisher, &motor_state_msg, NULL));
