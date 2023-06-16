#pragma once
#include <list>

// data buckets
typedef struct
{
  float orientation[4];
  float angular_velocity[3];
  float linear_acceleration[3];
} imu_data_t;

typedef struct
{
  float pose[3];  // TODO:
} odom_data_t;

// observers
template <typename DataQueueType>
class EventObserverInterface
{
public:
  virtual ~EventObserverInterface() {}
  friend class ImuRosEvent;

protected:
  virtual DataQueueType update() = 0;
};