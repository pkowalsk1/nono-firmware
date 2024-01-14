#pragma once

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <micro_ros_utilities/string_utilities.h>

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/float32_multi_array.h>

#include "uros/subjects.h"

#define RCCHECK(fn)                                           \
  {                                                           \
    rcl_ret_t temp_rc = fn;                                   \
    if ((temp_rc != RCL_RET_OK)) {                            \
      Serial.println("Error occured: " + temp_rc);            \
      while (1) {                                             \
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); \
        delay(300);                                           \
      }                                                       \
    }                                                         \
  }

#define RCSOFTCHECK(fn)                            \
  {                                                \
    rcl_ret_t temp_rc = fn;                        \
    if ((temp_rc != RCL_RET_OK)) {                 \
      Serial.println("Error occured: " + temp_rc); \
    }                                              \
  }

enum uros_state_t {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED,
};

class MicroROSWrapper
{
public:
  MicroROSWrapper(MicroROSWrapper & other) = delete;
  void operator=(const MicroROSWrapper &) = delete;

  static MicroROSWrapper * getInstance();

  void init(arduino::UART & uros_serial);
  void activate();
  void deactivate();
  void spinSome();

  uros_state_t evaluateConnectionState();
  uros_state_t getConnectionState() const { return connection_state_; }

protected:
  MicroROSWrapper(){};
  ~MicroROSWrapper(){};

private:
  void initImuMsg();
  void initJointStatesMsg();
  void initMotorsCmdMsg();

  static void imuTimerCallback(rcl_timer_t * timer, int64_t last_call_time);
  static void jointStateTimerCallback(rcl_timer_t * timer, int64_t last_call_time);
  static void motorsCmdCallback(const void * arg_input_message);

  static MicroROSWrapper * instance_;

  arduino::UART * uros_serial_;
  rclc_executor_t executor_;
  rclc_support_t support_;
  rcl_allocator_t allocator_;
  rcl_node_t node_;

  rcl_publisher_t imu_publisher_;
  rcl_publisher_t joint_state_publisher_;
  rcl_subscription_t motors_cmd_subscriber_;
  rcl_timer_t imu_timer_;
  rcl_timer_t joint_state_timer_;

  sensor_msgs__msg__Imu imu_msg_;
  sensor_msgs__msg__JointState joint_state_msg_;
  std_msgs__msg__Float32MultiArray motors_cmd_msg_;

  uros_state_t connection_state_ = WAITING_AGENT;

  static constexpr const char * node_name_ = "rpi_pico_node";
  static constexpr const char * node_namespace_ = "nono";
  static constexpr int imu_timer_freq_ = 70;
  static constexpr int joint_timer_freq_ = 50;
  static constexpr int motor_state_msg_len_ = 2;
};