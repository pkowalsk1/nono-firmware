#include <Arduino.h>
#include <Wire.h>

#include "gpio_cfg.h"
#include "imu.h"
#include "motors.h"
#include "uros/u_ros_cfg.h"

enum states
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

UART uRosSerial(UROS_SERIAL_TX, UROS_SERIAL_RX, NC, NC);

WheelMotorDriver left_motor_wheel(M1_PWM_A, M1_PWM_B, M1_ENC_A, M1_ENC_B, m1_default_dir);
WheelMotorDriver right_motor_wheel(M2_PWM_A, M2_PWM_B, M2_ENC_A, M2_ENC_B, m2_default_dir);
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

  gpio_set_function(UROS_SERIAL_TX, GPIO_FUNC_UART);
  gpio_set_function(UROS_SERIAL_RX, GPIO_FUNC_UART);

  Serial.begin(115200);
  uRosSerial.begin(576000);

  imu_bno.init();

  // setup uRos entities
  imu_timer_event->add(&imu_bno);
  joint_timer_event->add(&left_motor_wheel);
  motors_cmd_event->add(&left_motor_wheel);
  joint_timer_event->add(&right_motor_wheel);
  motors_cmd_event->add(&right_motor_wheel);

  set_microros_serial_transports(uRosSerial);
  delay(2000);

  state = WAITING_AGENT;

  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
}

void loop()
{
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE
                                                                                  : WAITING_AGENT;);
      Serial.println("Waiting for the agent...");
      break;
    case AGENT_AVAILABLE:
      Serial.println("Attempting to create node");
      state = (true == uRosCreateEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        uRosDestroyEntities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                                        ? AGENT_CONNECTED
                                        : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
      }
      break;
    case AGENT_DISCONNECTED:
      Serial.println("Agent disconnected");
      uRosDestroyEntities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }
}