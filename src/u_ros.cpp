#include "u_ros.h"

rcl_publisher_t imu_publisher;
rcl_publisher_t motor_state_publisher;

sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__JointState motor_state_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t imu_timer;

void uRosCreateEntity()
{
  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, NODE_NAME, NODE_NAMESPACE, &support));

  // create publishers
  RCCHECK(rclc_publisher_init_default(
    &imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data_raw"));

  RCCHECK(rclc_publisher_init_default(
    &motor_state_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "joint_state"));

  imuMsgInit(&imu_msg);
  motorStateMsgInit(&motor_state_msg);

  // create timer,
  RCCHECK(rclc_timer_init_default(
    &imu_timer, &support, RCL_MS_TO_NS(1 / IMU_TIME_FREQ), imuTimerCallback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &imu_timer));
}

void imuTimerCallback(rcl_timer_t* timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);

  if (timer != NULL) {
    // TODO: https://github.com/micro-ROS/micro_ros_platformio/tree/main#time-source
    if (rmw_uros_epoch_synchronized()) {
      imu_msg.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
      imu_msg.header.stamp.nanosec = rmw_uros_epoch_nanos();
    }

    // imu_msg.orientation.x = quat.x();
    // imu_msg.orientation.y = quat.y();
    // imu_msg.orientation.z = quat.z();
    // imu_msg.orientation.w = quat.w();

    // sensors_event_t* gyro = &angVelocityData;
    // imu_msg.angular_velocity.x = gyro->gyro.x;
    // imu_msg.angular_velocity.y = gyro->gyro.y;
    // imu_msg.angular_velocity.z = gyro->gyro.z;

    // sensors_event_t* acceleration = &linearAccelData;
    // imu_msg.linear_acceleration.x = acceleration->acceleration.x;
    // imu_msg.linear_acceleration.y = acceleration->acceleration.y;
    // imu_msg.linear_acceleration.z = acceleration->acceleration.z;

    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
  }
}

void imuMsgInit(sensor_msgs__msg__Imu* arg_message)
{
  arg_message->header.frame_id.data = (char*)"imu_link";
  arg_message->orientation_covariance[0] = 0.05;
  arg_message->orientation_covariance[4] = 0.05;
  arg_message->orientation_covariance[8] = 0.05;
  arg_message->angular_velocity_covariance[0] = 0.05;
  arg_message->angular_velocity_covariance[4] = 0.05;
  arg_message->angular_velocity_covariance[8] = 0.05;
  arg_message->linear_acceleration_covariance[0] = 0.05;
  arg_message->linear_acceleration_covariance[4] = 0.05;
  arg_message->linear_acceleration_covariance[8] = 0.05;
}

void motorStateMsgInit(sensor_msgs__msg__JointState* arg_message)
{
  static rosidl_runtime_c__String msg_name_tab[MOT_STATE_MSG_LEN];
  static double msg_data_tab[3][MOT_STATE_MSG_LEN];
  char* frame_id = (char*)"wheel_joints_state";

  arg_message->position.data = msg_data_tab[0];
  arg_message->position.capacity = arg_message->position.size = MOT_STATE_MSG_LEN;
  arg_message->velocity.data = msg_data_tab[1];
  arg_message->velocity.capacity = arg_message->velocity.size = MOT_STATE_MSG_LEN;
  arg_message->effort.data = msg_data_tab[2];
  arg_message->effort.capacity = arg_message->effort.size = MOT_STATE_MSG_LEN;
  arg_message->header.frame_id.data = frame_id;
  arg_message->header.frame_id.capacity = arg_message->header.frame_id.size =
    strlen((const char*)frame_id);
  msg_name_tab->capacity = msg_name_tab->size = MOT_STATE_MSG_LEN;
  msg_name_tab[0].data = (char*)"left_wheel_joint";
  msg_name_tab[1].data = (char*)"right_wheel_joint";

  for (uint8_t i = 0; i < MOT_STATE_MSG_LEN; i++) {
    msg_name_tab[i].capacity = msg_name_tab[i].size = strlen(msg_name_tab[i].data);
  }

  arg_message->name.capacity = arg_message->name.size = MOT_STATE_MSG_LEN;
  arg_message->name.data = msg_name_tab;
}

void errorLoop()
{
  while (1) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
  }
}