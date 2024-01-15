#pragma once

typedef struct
{
  float orientation[4];
  float angular_velocity[3];
  float linear_acceleration[3];
} imu_data_t;

typedef struct
{
  double actual_ang_pose;
  double actual_ang_vel;
} joint_states_data_t;

typedef double motors_cmd_data_t;

template <typename DataQueueType>
class EventObserverInterface
{
public:
  virtual ~EventObserverInterface() = default;
  virtual void update(DataQueueType & data_queue) = 0;
};