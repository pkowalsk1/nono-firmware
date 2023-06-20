#include "motors.h"
#include "uros/u_ros_cfg.h"

/* ROS publishers */
rcl_publisher_t imu_publisher;
rcl_publisher_t joint_states_publisher;

/* ROS subscribers */
rcl_subscription_t motors_cmd_subscriber;

/* ROS timers */
rcl_timer_t imu_timer;
rcl_timer_t joint_states_timer;

/* ROS msgs */
sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__JointState joint_states_msg;
std_msgs__msg__Float32MultiArray motors_cmd_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

ImuRosEvent* imu_timer_event = new ImuRosEvent();
JointPubRosEvent* joint_timer_event = new JointPubRosEvent();
MotorsCmdRosEvent* motors_cmd_event = new MotorsCmdRosEvent();

void uRosCreateEntities()
{
  size_t ros_handles_cnt = 0;

  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, NODE_NAME, NODE_NAMESPACE, &support));

  // create publishers
  RCCHECK(rclc_publisher_init_best_effort(
    &imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data_raw"));

  RCCHECK(rclc_publisher_init_best_effort(
    &joint_states_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "joint_states"));

  imuMsgInit(&imu_msg);
  jointStatesMsgInit(&joint_states_msg);
  motorStateMsgInit(&motors_cmd_msg);

  // create subscribers
  RCCHECK(rclc_subscription_init_best_effort(
    &motors_cmd_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "motors_cmd"));
  ros_handles_cnt++;

  // create timer
  RCCHECK(rclc_timer_init_default(
    &imu_timer, &support, RCL_MS_TO_NS(1000 / IMU_TIMER_FREQ), imuTimerCallback));
  ros_handles_cnt++;

  RCCHECK(rclc_timer_init_default(
    &joint_states_timer, &support, RCL_MS_TO_NS(1000 / JOINT_TIMER_FREQ),
    jointStatesTimerCallback));
  ros_handles_cnt++;

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, ros_handles_cnt, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &imu_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &joint_states_timer));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &motors_cmd_subscriber, &motors_cmd_msg, &motorsCmdCallback, ON_NEW_DATA));
}

void imuTimerCallback(rcl_timer_t* timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);

  if (timer != NULL) {
    // TODO: https://github.com/micro-ROS/micro_ros_platformio/tree/main#time-source
    if (rmw_uros_epoch_synchronized()) {
      imu_msg.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
      imu_msg.header.stamp.nanosec = rmw_uros_epoch_nanos();
    } else {
      int64_t u_sec = time_us_64();
      joint_states_msg.header.stamp.sec = u_sec / (int64_t)1e6;
      joint_states_msg.header.stamp.nanosec = u_sec % (int64_t)1e6;
    }

    imu_timer_event->notify();
    imu_data_t imu_queue = imu_timer_event->getDataQueue()[0];

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

void jointStatesTimerCallback(rcl_timer_t* timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);

  if (timer != NULL) {
    if (rmw_uros_epoch_synchronized()) {
      joint_states_msg.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
      joint_states_msg.header.stamp.nanosec = rmw_uros_epoch_nanos();
    } else {
      int64_t u_sec = time_us_64();
      joint_states_msg.header.stamp.sec = u_sec / (int64_t)1e6;
      joint_states_msg.header.stamp.nanosec = u_sec % (int64_t)1e6;
    }

    joint_timer_event->notify();
    std::vector<joint_states_data_t> joint_states_queue = joint_timer_event->getDataQueue();

    for (size_t i = 0; i < joint_states_queue.size(); i++) {
      joint_states_msg.position.data[i] = joint_states_queue[i].actual_ang_pose;
      joint_states_msg.velocity.data[i] = joint_states_queue[i].actual_ang_vel;
    }

    RCSOFTCHECK(rcl_publish(&joint_states_publisher, &joint_states_msg, NULL));
  }
}

void motorsCmdCallback(const void* arg_input_message)
{
  static std_msgs__msg__Float32MultiArray* setpoint_msg;
  setpoint_msg = (std_msgs__msg__Float32MultiArray*)arg_input_message;

  if (setpoint_msg->data.size == 2) {
    std::vector<motors_cmd_data_t> motors_cmd_data{
      setpoint_msg->data.data[0], setpoint_msg->data.data[1]};
    motors_cmd_event->setDataQueue(motors_cmd_data);
  }
  motors_cmd_event->notify();
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

void jointStatesMsgInit(sensor_msgs__msg__JointState* arg_message)
{
  static rosidl_runtime_c__String msg_name_tab[MOT_STATE_MSG_LEN];
  static double msg_data_tab[3][MOT_STATE_MSG_LEN];
  char* frame_id = (char*)"wheel_joint_states";

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

void motorStateMsgInit(std_msgs__msg__Float32MultiArray* arg_message)
{
  static float data[2] = {0, 0};
  arg_message->data.capacity = 2;
  arg_message->data.size = 2;
  arg_message->data.data = (float*)data;
}

void errorLoop()
{
  while (1) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
  }
}