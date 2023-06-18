#include "motors.h"
#include "uros/u_ros_cfg.h"

rcl_publisher_t imu_publisher;
rcl_publisher_t joint_state_publisher;

sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__JointState joint_state_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t imu_timer;
rcl_timer_t joint_state_timer;

ImuRosEvent* imu_timer_event = new ImuRosEvent();
extern WheelMotorDriver left_motor_wheel;
extern WheelMotorDriver right_motor_wheel;

void uRosCreateEntities()
{
  size_t ros_handles_cnt = 0;

  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, NODE_NAME, NODE_NAMESPACE, &support));

  // create publishers
  RCCHECK(rclc_publisher_init_default(
    &imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data_raw"));

  RCCHECK(rclc_publisher_init_default(
    &joint_state_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "joint_state"));

  imuMsgInit(&imu_msg);
  motorStateMsgInit(&joint_state_msg);

  // create timer,
  RCCHECK(rclc_timer_init_default(
    &imu_timer, &support, RCL_MS_TO_NS(1000 / IMU_TIMER_FREQ), imuTimerCallback));
  ros_handles_cnt++;

  RCCHECK(rclc_timer_init_default(
    &joint_state_timer, &support, RCL_MS_TO_NS(1000 / JOINT_TIMER_FREQ), jointStateTimerCallback));
  ros_handles_cnt++;

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, ros_handles_cnt, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &imu_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &joint_state_timer));
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

    imu_timer_event->notify();
    imu_data_t imu_queue = imu_timer_event->getImuDataQueue();

    imu_msg.orientation.x = imu_queue.orientation[0];
    imu_msg.orientation.y = imu_queue.orientation[1];
    imu_msg.orientation.z = imu_queue.orientation[2];
    imu_msg.orientation.w = imu_queue.orientation[3];
    imu_msg.angular_velocity.x = imu_queue.angular_velocity[0];
    imu_msg.angular_velocity.y = imu_queue.angular_velocity[1];
    imu_msg.angular_velocity.z = imu_queue.angular_velocity[2];
    imu_msg.linear_acceleration.x = imu_queue.linear_acceleration[0];
    imu_msg.linear_acceleration.y = imu_queue.linear_acceleration[1];
    imu_msg.linear_acceleration.z = imu_queue.linear_acceleration[2];

    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
  }
}

void jointStateTimerCallback(rcl_timer_t* timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);

  if (timer != NULL) {
    if (rmw_uros_epoch_synchronized()) {
      joint_state_msg.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
      joint_state_msg.header.stamp.nanosec = rmw_uros_epoch_nanos();
    }

    joint_state_msg.header.stamp.sec = time_us_64() / (int64_t)1e6;

    joint_state_msg.position.data[0] = left_motor_wheel.angPoseUpdate();
    joint_state_msg.position.data[1] = right_motor_wheel.angPoseUpdate();
    joint_state_msg.velocity.data[0] = left_motor_wheel.angVelUpdate();
    joint_state_msg.velocity.data[1] = right_motor_wheel.angVelUpdate();
    RCSOFTCHECK(rcl_publish(&joint_state_publisher, &joint_state_msg, NULL));
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
  char* frame_id = (char*)"wheel_joint_state";

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