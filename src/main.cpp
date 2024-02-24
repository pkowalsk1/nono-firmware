#include <Arduino.h>
#include <Wire.h>

#include "hardware/rosc.h"
#include "pico/sleep.h"

#include <SoftwareSerial.h>
#include <stdio.h>
#include "hardware/uart.h"
#include "pico/stdlib.h"

#include "gpio_cfg.h"
#include "imu.h"
#include "motors.h"
#include "uros/u_ros.h"

WheelMotorDriver left_motor_wheel(M1_PWM_A, M1_PWM_B, M1_ENC_A, M1_ENC_B, m1_default_dir);
WheelMotorDriver right_motor_wheel(M2_PWM_A, M2_PWM_B, M2_ENC_A, M2_ENC_B, m2_default_dir);
ImuDriver imu_bno(55, 0x29);

MicroROSWrapper * uros_wrapper = MicroROSWrapper::getInstance();

volatile bool shutdown_btn_pressed = false;

void setup()
{
  for (const GPIOPinDirection & gpio : pin_directions) {
    pinMode(gpio.pin_number, gpio.direction);
  }

  for (const GPIOPinFunction & gpio : pin_functions) {
    gpio_set_function(gpio.pin_number, gpio.function);
  }

  attachInterrupt(
    digitalPinToInterrupt(M1_ENC_A), []() { left_motor_wheel.readEncoder(); }, RISING);
  attachInterrupt(
    digitalPinToInterrupt(M2_ENC_A), []() { right_motor_wheel.readEncoder(); }, RISING);
  // attachInterrupt(
  //   digitalPinToInterrupt(POWER_BTN), []() { shutdown_btn_pressed = true; }, FALLING);

  Serial.begin(115200);

  Serial2.setRX(UROS_SERIAL_RX_);
  Serial2.setTX(UROS_SERIAL_TX_);
  Serial2.begin(576000);
  Wire.begin();

  imu_bno.init();

  uros_wrapper->init(Serial2);
  uros_wrapper->addImuObserver(&imu_bno);
  uros_wrapper->addJointStateObserver(&left_motor_wheel);
  uros_wrapper->addJointStateObserver(&right_motor_wheel);
  uros_wrapper->addCmdObserver(&left_motor_wheel);
  uros_wrapper->addCmdObserver(&right_motor_wheel);

  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
}

void loop()
{
  unsigned long start_time = 0;

  while (true) {
    uros_state_t connection_state = uros_wrapper->evaluateConnectionState();

    switch (connection_state) {
      case WAITING_AGENT:
        Serial.println("Waiting for the agent...");
        delay(500);
        break;

      case AGENT_AVAILABLE:
        Serial.println("Attempting to create node...");
        uros_wrapper->activate();

        Serial.println("Node created successfully.");
        start_time = millis();
        break;

      case AGENT_CONNECTED:
        uros_wrapper->spinSome();
        break;

      case AGENT_DISCONNECTED:
        Serial.println("Agent disconnected. Deactivating...");
        uros_wrapper->deactivate();
        break;

      default:
        break;
    }

    if (start_time != 0 && millis() - start_time >= 5000) {
      uros_wrapper->deactivate();

      digitalWrite(LED_BUILTIN, LOW);
      delay(500);
      digitalWrite(LED_BUILTIN, HIGH);

      Serial2.end();

      sleep_run_from_xosc();
      sleep_goto_dormant_until_edge_high(POWER_BTN);

      gpio_set_function(UROS_SERIAL_RX_, GPIO_FUNC_UART);
      gpio_set_function(UROS_SERIAL_TX_, GPIO_FUNC_UART);
      delay(10);
      Serial2.setRX(UROS_SERIAL_RX_);
      Serial2.setTX(UROS_SERIAL_TX_);
      delay(10);
      Serial2.begin(576000);
      Serial.begin(115200);

      digitalWrite(LED_BUILTIN, LOW);

      start_time = 0;
    }
  }
}