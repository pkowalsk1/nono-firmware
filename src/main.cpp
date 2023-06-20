#include <Arduino.h>
#include <Wire.h>

#include "gpio_cfg.h"
#include "imu.h"
#include "motors.h"
#include "uros/u_ros_cfg.h"

WheelMotorDriver left_motor_wheel(M1_PWM_A, M1_PWM_B, M1_ENC_A, M1_ENC_B, M1_DEFAULT_DIR, 0);
WheelMotorDriver right_motor_wheel(M2_PWM_A, M2_PWM_B, M2_ENC_A, M2_ENC_B, M2_DEFAULT_DIR, 1);
ImuDriver imu_bno(55, 0x29);

extern rclc_executor_t executor;
extern ImuRosEvent* imu_timer_event;
extern JointPubRosEvent* joint_timer_event;
extern MotorsCmdRosEvent* motors_cmd_event;

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(M1_PWM_A, OUTPUT);
  pinMode(M1_PWM_B, OUTPUT);
  pinMode(M2_PWM_A, OUTPUT);
  pinMode(M2_PWM_B, OUTPUT);

  pinMode(M1_ENC_A, INPUT);
  pinMode(M1_ENC_B, INPUT);
  attachInterrupt(
    digitalPinToInterrupt(M1_ENC_A), []() { left_motor_wheel.readEncoder(); }, RISING);

  pinMode(M2_ENC_A, INPUT);
  pinMode(M2_ENC_B, INPUT);
  attachInterrupt(
    digitalPinToInterrupt(M2_ENC_A), []() { right_motor_wheel.readEncoder(); }, RISING);

  Serial.begin(115200);

  imu_bno.init();

  // setup ros entities
  imu_timer_event->add(&imu_bno);
  joint_timer_event->add(&left_motor_wheel);
  motors_cmd_event->add(&left_motor_wheel);
  joint_timer_event->add(&right_motor_wheel);
  motors_cmd_event->add(&right_motor_wheel);

  uRosCreateEntities();

  digitalWrite(LED_BUILTIN, HIGH);

  delay(100);
}

void loop()
{
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}