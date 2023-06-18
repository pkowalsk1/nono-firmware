#pragma once

typedef struct
{
  float orientation[4];
  float angular_velocity[3];
  float linear_acceleration[3];
} imu_data_t;

typedef struct
{
  double actual_ang_pose[2];
  double actual_ang_vel[2];
  uint64_t last_cmd_req_time;
} joint_data_t;

template <typename DataQueueType>
class EventObserverInterface
{
public:
  virtual ~EventObserverInterface() {}
  friend class ImuRosEvent;
  friend class JointControlRosEvent;

protected:
  virtual void update(DataQueueType& data_queue) = 0;
};