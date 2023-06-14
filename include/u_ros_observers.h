#ifndef U_ROS_OBSERVERS_H
#define U_ROS_OBSERVERS_H

#include <list>
#include "u_ros_subjects.h"

typedef struct
{
  float orientation[4];
  float angular_velocity[3];
  float linear_acceleration[3];
} imu_data_t;

class EventObserverInterface
{
public:
  virtual ~EventObserverInterface() {}
  friend class RosEvent;

protected:
  virtual void update() = 0;
  static int unique_observers_;
};

int EventObserverInterface::unique_observers_ = 0;

#endif