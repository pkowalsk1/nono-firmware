#include <Arduino.h>
#include <Wire.h>

#include "pico/multicore.h"

#include "gpio_cfg.h"
#include "imu.h"
#include "motors.h"
#include "uros/u_ros_cfg.h"

UART uRosSerial(UROS_SERIAL_TX_, UROS_SERIAL_RX_, NC, NC);
uros_state_t uros_state;

WheelMotorDriver left_motor_wheel(M1_PWM_A, M1_PWM_B, M1_ENC_A, M1_ENC_B, m1_default_dir);
WheelMotorDriver right_motor_wheel(M2_PWM_A, M2_PWM_B, M2_ENC_A, M2_ENC_B, m2_default_dir);
ImuDriver imu_bno(55, 0x29);

extern rclc_executor_t executor;
extern ImuRosEvent* imu_timer_event;
extern JointPubRosEvent* joint_timer_event;
extern MotorsCmdRosEvent* motors_cmd_event;

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
  for (const PinModeInfo& pin : pin_map_gpio) {
    pinMode(pin.gpio, pin.mode);
  }

  for (const PinFncInfo& pin : pin_map_fnc_gpio) {
    gpio_set_function(pin.gpio, pin.fnc);
  }

  attachInterrupt(
    digitalPinToInterrupt(M1_ENC_A), []() { left_motor_wheel.readEncoder(); }, RISING);
  attachInterrupt(
    digitalPinToInterrupt(M2_ENC_A), []() { right_motor_wheel.readEncoder(); }, RISING);
  attachInterrupt(
    digitalPinToInterrupt(POWER_BTN), []() { shutdown_btn_pressed = true; }, FALLING);

  Wire.begin();

  Serial.begin(115200);
  uRosSerial.begin(576000);

  imu_bno.init();

  multicore_launch_core1(shutdown_handler);

  // setup uRos entities
  imu_timer_event->add(&imu_bno);
  joint_timer_event->add(&left_motor_wheel);
  motors_cmd_event->add(&left_motor_wheel);
  joint_timer_event->add(&right_motor_wheel);
  motors_cmd_event->add(&right_motor_wheel);

  set_microros_serial_transports(uRosSerial);
  delay(2000);

  uros_state = WAITING_AGENT;

  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
}

void loop()
{
  switch (uros_state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, uros_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                                             ? AGENT_AVAILABLE
                                             : WAITING_AGENT;);
      Serial.println("Waiting for the agent...");
      break;
    case AGENT_AVAILABLE:
      Serial.println("Attempting to create node");
      uros_state = (true == uRosCreateEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (uros_state == WAITING_AGENT) {
        uRosDestroyEntities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, uros_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                                             ? AGENT_CONNECTED
                                             : AGENT_DISCONNECTED;);
      if (uros_state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
      }
      break;
    case AGENT_DISCONNECTED:
      Serial.println("Agent disconnected");
      uRosDestroyEntities();
      uros_state = WAITING_AGENT;
      break;
    default:
      break;
  }
}