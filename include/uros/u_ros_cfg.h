#pragma once
#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/float32_multi_array.h>

#include "uros/subjects.h"

#define NODE_NAME "rpi_pico_node"
#define NODE_NAMESPACE "nono"
#define MOT_STATE_MSG_LEN 2

#define IMU_TIMER_FREQ 50    // Hz
#define JOINT_TIMER_FREQ 20  // HZ

#define RCCHECK(fn)                \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
      errorLoop();                 \
    }                              \
  }
#define RCSOFTCHECK(fn)            \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
    }                              \
  }

void uRosCreateEntities();
void imuTimerCallback(rcl_timer_t* timer, int64_t last_call_time);
void jointStatesTimerCallback(rcl_timer_t* timer, int64_t last_call_time);
void imuMsgInit(sensor_msgs__msg__Imu* arg_message);
void jointStatesMsgInit(sensor_msgs__msg__JointState * arg_message);
void motorsCmdCallback(const void* arg_input_message);
void motorStateMsgInit(std_msgs__msg__Float32MultiArray* arg_message);

/**
  TODO: add description.
  
  @param x first quantity to multiply.
  @param y second quantity to multiply.
  @return result of the multiplication operation
*/
void errorLoop();