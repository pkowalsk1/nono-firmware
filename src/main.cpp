#include <Arduino.h>
#include <Wire.h>

#include "gpio_cfg.h"
#include "imu.h"
#include "u_ros.h"
#include "motors.h"

WheelMotorDriver left_motor_wheel = WheelMotorDriver(M1_PWM_A, M1_PWM_B, M1_ENC_A, M1_ENC_B);
WheelMotorDriver right_motor_wheel = WheelMotorDriver(M2_PWM_A, M2_PWM_B, M2_ENC_A, M2_ENC_B);
ImuDriver imu_bno = ImuDriver(55, 0x29);

extern rclc_executor_t executor;

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
  
  uRosCreateEntity();

  digitalWrite(LED_BUILTIN, HIGH);

  delay(100);
}

void loop()
{
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}