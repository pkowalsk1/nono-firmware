#ifndef U_ROS_H
#define U_ROS_H

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>

#include "u_ros_observers.h"
#include "u_ros_subjects.h"

#define NODE_NAME "rpi_pico_node"
#define NODE_NAMESPACE "nono"
#define MOT_STATE_MSG_LEN 2

#define IMU_TIME_FREQ 50  // Hz

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


void uRosCreateEntity();
void imuTimerCallback(rcl_timer_t* timer, int64_t last_call_time);
void imuMsgInit(sensor_msgs__msg__Imu* arg_message);
void motorStateMsgInit(sensor_msgs__msg__JointState* arg_message);

/**
  TODO: add description.
  
  @param x first quantity to multiply.
  @param y second quantity to multiply.
  @return result of the multiplication operation
*/
void errorLoop();

#endif /* MICRO_ROS_H */