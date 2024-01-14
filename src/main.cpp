#include <Arduino.h>
#include <Wire.h>

#include "pico/multicore.h"

#include "gpio_cfg.h"
#include "imu.h"
#include "motors.h"
#include "uros/u_ros_cfg.h"

WheelMotorDriver left_motor_wheel(M1_PWM_A, M1_PWM_B, M1_ENC_A, M1_ENC_B, m1_default_dir);
WheelMotorDriver right_motor_wheel(M2_PWM_A, M2_PWM_B, M2_ENC_A, M2_ENC_B, m2_default_dir);
ImuDriver imu_bno(55, 0x29);

arduino::UART uros_serial(UROS_SERIAL_TX_, UROS_SERIAL_RX_, NC, NC);
MicroROSWrapper * uros_wrapper = MicroROSWrapper::getInstance();

volatile bool shutdown_btn_pressed = false;

void shutdown_handler()
{
  while (true) {
    if (shutdown_btn_pressed) {
      digitalWrite(LED_BUILTIN, LOW);
      sleep_ms(200);
      digitalWrite(LED_BUILTIN, HIGH);
      sleep_ms(1000);
      digitalWrite(LED_BUILTIN, LOW);
      sleep_ms(500);
      digitalWrite(LED_BUILTIN, HIGH);
    }

    sleep_ms(200);
  }
}

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
  attachInterrupt(
    digitalPinToInterrupt(POWER_BTN), []() { shutdown_btn_pressed = true; }, FALLING);
  multicore_launch_core1(shutdown_handler);

  Serial.begin(115200);
  uros_serial.begin(576000);
  Wire.begin();

  imu_bno.init();
  uros_wrapper->init(uros_serial);

  // setup uRos entities
  // imu_timer_event->add(&imu_bno);
  // joint_timer_event->add(&left_motor_wheel);
  // motors_cmd_event->add(&left_motor_wheel);
  // joint_timer_event->add(&right_motor_wheel);
  // motors_cmd_event->add(&right_motor_wheel);

  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
}

void loop()
{
  switch (uros_wrapper->getConnectionState()) {
    case WAITING_AGENT:
      Serial.println("Waiting for the agent...");
      uros_wrapper->evaluateConnectionState();

      // TODO: use other method of delay
      delay(500);
      break;

    case AGENT_AVAILABLE:
      Serial.println("Attempting to create node");
      uros_wrapper->activate();

      if (uros_wrapper->getConnectionState() == WAITING_AGENT) {
        Serial.println("Creating node failed.");
        uros_wrapper->deactivate();
      };
      break;

    case AGENT_CONNECTED:
      uros_wrapper->evaluateConnectionState();
      if (uros_wrapper->getConnectionState() == AGENT_CONNECTED) {
        Serial.println("Spin");
        uros_wrapper->spinSome();
      }
      break;

    case AGENT_DISCONNECTED:
      Serial.println("Agent disconnected");
      uros_wrapper->deactivate();
      break;

    default:
      break;
  }
}