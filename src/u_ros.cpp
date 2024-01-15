#include <algorithm>

#include "uros/u_ros.h"

template <typename DataQueueType>
void MicroROSEvent<DataQueueType>::addObserver(EventObserverInterface<DataQueueType> * obsv)
{
  obsvs_.push_back(obsv);
  data_queue_.resize(obsvs_.size());
}

template <typename DataQueueType>
void MicroROSEvent<DataQueueType>::notify()
{
  if (obsvs_.empty()) return;

  for (size_t i = 0; i < obsvs_.size(); i++) {
    obsvs_[i]->update(data_queue_[i]);
  }
}

MicroROSWrapper * MicroROSWrapper::instance_{nullptr};

MicroROSWrapper * MicroROSWrapper::getInstance()
{
  if (instance_ == nullptr) {
    instance_ = new MicroROSWrapper();
  }
  return instance_;
}

void MicroROSWrapper::init(arduino::UART & uros_serial)
{
  uros_serial_ = &uros_serial;

  set_microros_serial_transports(*uros_serial_);
  delay(2000);
}

void MicroROSWrapper::activate()
{
  size_t ros_handles_cnt = 0;

  allocator_ = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support_, 0, NULL, &allocator_));
  RCCHECK(rclc_node_init_default(&node_, node_name_, node_namespace_, &support_));

  initImuMsg();
  initJointStatesMsg();
  initMotorsCmdMsg();

  RCCHECK(rclc_publisher_init_best_effort(
    &imu_publisher_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data_raw"));
  RCCHECK(rclc_publisher_init_best_effort(
    &joint_state_publisher_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "joint_state"));
  RCCHECK(rclc_subscription_init_best_effort(
    &motors_cmd_subscriber_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "motor_cmd"));
  ros_handles_cnt++;

  RCCHECK(rclc_timer_init_default(
    &imu_timer_, &support_, RCL_MS_TO_NS(1000 / imu_timer_freq_),
    &MicroROSWrapper::imuTimerCallback));
  ros_handles_cnt++;

  RCCHECK(rclc_timer_init_default(
    &joint_state_timer_, &support_, RCL_MS_TO_NS(1000 / joint_timer_freq_),
    &MicroROSWrapper::jointStateTimerCallback));
  ros_handles_cnt++;

  RCCHECK(rclc_executor_init(&executor_, &support_.context, ros_handles_cnt, &allocator_));
  RCCHECK(rclc_executor_add_timer(&executor_, &imu_timer_));
  RCCHECK(rclc_executor_add_timer(&executor_, &joint_state_timer_));
  RCCHECK(rclc_executor_add_subscription(
    &executor_, &motors_cmd_subscriber_, &motors_cmd_msg_, &MicroROSWrapper::motorsCmdCallback,
    ON_NEW_DATA));

  while (!rmw_uros_epoch_synchronized()) {
    RCCHECK(rmw_uros_sync_session(100));
  }

  connection_state_ = AGENT_CONNECTED;
}

void MicroROSWrapper::deactivate()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support_.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  micro_ros_string_utilities_destroy(&imu_msg_.header.frame_id);
  micro_ros_string_utilities_destroy(&joint_state_msg_.header.frame_id);
  micro_ros_string_utilities_destroy(&joint_state_msg_.name.data[0]);
  micro_ros_string_utilities_destroy(&joint_state_msg_.name.data[1]);

  RCSOFTCHECK(rcl_publisher_fini(&joint_state_publisher_, &node_));
  RCSOFTCHECK(rcl_publisher_fini(&imu_publisher_, &node_));

  RCSOFTCHECK(rcl_timer_fini(&imu_timer_));
  RCSOFTCHECK(rcl_timer_fini(&joint_state_timer_));

  RCSOFTCHECK(rclc_executor_fini(&executor_));
  RCSOFTCHECK(rcl_node_fini(&node_));
  RCSOFTCHECK(rclc_support_fini(&support_));

  connection_state_ = WAITING_AGENT;
}

void MicroROSWrapper::spinSome()
{
  if (connection_state_ != AGENT_CONNECTED) {
    RCCHECK(RCL_RET_ERROR);
  }

  rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(100));
}

uros_state_t MicroROSWrapper::evaluateConnectionState()
{
  const bool res = (RMW_RET_OK == rmw_uros_ping_agent(100, 1));

  connection_state_ =
    res ? (connection_state_ == WAITING_AGENT ? AGENT_AVAILABLE : AGENT_CONNECTED)
        : (connection_state_ == AGENT_CONNECTED ? AGENT_DISCONNECTED : WAITING_AGENT);

  return connection_state_;
}

void MicroROSWrapper::addImuObserver(EventObserverInterface<imu_data_t> * obsv)
{
  imu_timer_event_.addObserver(obsv);
}

void MicroROSWrapper::addJointStateObserver(EventObserverInterface<joint_states_data_t> * obsv)
{
  joint_timer_event_.addObserver(obsv);
}

void MicroROSWrapper::addCmdObserver(EventObserverInterface<motors_cmd_data_t> * obsv)
{
  motors_cmd_event_.addObserver(obsv);
}

void MicroROSWrapper::imuTimerCallback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  MicroROSWrapper * instance = getInstance();

  if (timer != NULL) {
    if (rmw_uros_epoch_synchronized()) {
      instance->imu_msg_.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
      instance->imu_msg_.header.stamp.nanosec = rmw_uros_epoch_nanos();
    } else {
      int64_t u_sec = time_us_64();
      instance->imu_msg_.header.stamp.sec = u_sec / (int64_t)1e6;
      instance->imu_msg_.header.stamp.nanosec = u_sec % (int64_t)1e6;
    }

    instance->imu_timer_event_.notify();
    imu_data_t imu_queue = instance->imu_timer_event_.getDataQueue()[0];

    instance->imu_msg_.orientation.x = imu_queue.orientation[0];
    instance->imu_msg_.orientation.y = imu_queue.orientation[1];
    instance->imu_msg_.orientation.z = imu_queue.orientation[2];
    instance->imu_msg_.orientation.w = imu_queue.orientation[3];
    instance->imu_msg_.angular_velocity.x = imu_queue.angular_velocity[0];
    instance->imu_msg_.angular_velocity.y = imu_queue.angular_velocity[1];
    instance->imu_msg_.angular_velocity.z = imu_queue.angular_velocity[2];
    instance->imu_msg_.linear_acceleration.x = imu_queue.linear_acceleration[0];
    instance->imu_msg_.linear_acceleration.y = imu_queue.linear_acceleration[1];
    instance->imu_msg_.linear_acceleration.z = imu_queue.linear_acceleration[2];

    RCSOFTCHECK(rcl_publish(&instance->imu_publisher_, &instance->imu_msg_, NULL));
  }
}

void MicroROSWrapper::jointStateTimerCallback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  MicroROSWrapper * instance = getInstance();

  if (timer != NULL) {
    if (rmw_uros_epoch_synchronized()) {
      instance->joint_state_msg_.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
      instance->joint_state_msg_.header.stamp.nanosec = rmw_uros_epoch_nanos();
    } else {
      int64_t u_sec = time_us_64();
      instance->joint_state_msg_.header.stamp.sec = u_sec / (int64_t)1e6;
      instance->joint_state_msg_.header.stamp.nanosec = u_sec % (int64_t)1e6;
    }

    instance->joint_timer_event_.notify();
    std::vector<joint_states_data_t> joint_states_queue =
      instance->joint_timer_event_.getDataQueue();

    for (size_t i = 0; i < joint_states_queue.size(); i++) {
      instance->joint_state_msg_.position.data[i] = joint_states_queue[i].actual_ang_pose;
      instance->joint_state_msg_.velocity.data[i] = joint_states_queue[i].actual_ang_vel;
    }

    RCSOFTCHECK(rcl_publish(&instance->joint_state_publisher_, &instance->joint_state_msg_, NULL));
  }
}

void MicroROSWrapper::motorsCmdCallback(const void * arg_input_message)
{
  MicroROSWrapper * instance = getInstance();

  static std_msgs__msg__Float32MultiArray * setpoint_msg;
  setpoint_msg = (std_msgs__msg__Float32MultiArray *)arg_input_message;

  if (setpoint_msg->data.size != 2) {
    return;
  }

  std::vector<motors_cmd_data_t> motors_cmd_data{
    setpoint_msg->data.data[0], setpoint_msg->data.data[1]};

  instance->motors_cmd_event_.setDataQueue(motors_cmd_data);
  instance->motors_cmd_event_.notify();
}

void MicroROSWrapper::initImuMsg()
{
  imu_msg_.header.frame_id = micro_ros_string_utilities_init("imu_link");

  std::fill(
    std::begin(imu_msg_.orientation_covariance), std::end(imu_msg_.orientation_covariance), 0);
  std::fill(
    std::begin(imu_msg_.angular_velocity_covariance),
    std::end(imu_msg_.angular_velocity_covariance), 0);
  std::fill(
    std::begin(imu_msg_.linear_acceleration_covariance),
    std::end(imu_msg_.linear_acceleration_covariance), 0);

  // Set diagonal values to 0.05
  for (int i = 0; i < 9; i += 4) {
    imu_msg_.orientation_covariance[i] = 0.05;
    imu_msg_.angular_velocity_covariance[i] = 0.05;
    imu_msg_.linear_acceleration_covariance[i] = 0.05;
  }
}

void MicroROSWrapper::initJointStatesMsg()
{
  joint_state_msg_.header.frame_id = micro_ros_string_utilities_init("base_link");
  joint_state_msg_.name.data =
    (rosidl_runtime_c__String *)malloc(3 * sizeof(rosidl_runtime_c__String));
  joint_state_msg_.name.size = 2;
  joint_state_msg_.name.capacity = 2;
  joint_state_msg_.name.data[0] = micro_ros_string_utilities_init("left_wheel");
  joint_state_msg_.name.data[1] = micro_ros_string_utilities_init("right_wheel");

  joint_state_msg_.position.data = (double *)malloc(2 * sizeof(double));
  joint_state_msg_.position.size = 2;
  joint_state_msg_.position.capacity = 2;
  joint_state_msg_.position.data[0] = 0.0;
  joint_state_msg_.position.data[1] = 0.0;

  joint_state_msg_.velocity.data = (double *)malloc(2 * sizeof(double));
  joint_state_msg_.velocity.size = 2;
  joint_state_msg_.velocity.capacity = 2;
  joint_state_msg_.velocity.data[0] = 0.0;
  joint_state_msg_.velocity.data[1] = 0.0;

  joint_state_msg_.effort.data = (double *)malloc(2 * sizeof(double));
  joint_state_msg_.effort.size = 2;
  joint_state_msg_.effort.capacity = 2;
  joint_state_msg_.effort.data[0] = 0.0;
  joint_state_msg_.effort.data[1] = 0.0;
}

void MicroROSWrapper::initMotorsCmdMsg()
{
  static float data[2] = {0, 0};
  motors_cmd_msg_.data.capacity = 2;
  motors_cmd_msg_.data.size = 2;
  motors_cmd_msg_.data.data = (float *)data;
}