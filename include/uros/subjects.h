#pragma once
#include <list>

#include "uros/observers.h"

// events
template <typename DataQueueType>
class RosEventInterface
{
public:
  virtual ~RosEventInterface(){};

  void add(EventObserverInterface<DataQueueType>* obsv)
  {
    obsvs_.push_back(obsv);
  };
  void remove(EventObserverInterface<DataQueueType>* obsv)
  {
    obsvs_.remove(obsv);
  };
  virtual void notify() = 0;

protected:
  std::list<EventObserverInterface<DataQueueType>*> obsvs_;
};

class ImuRosEvent : public RosEventInterface<imu_data_t>
{
public:
  void notify() override
  {
    if (obsvs_.empty()) return;
    for (auto observer : obsvs_) {
      imu_data_queue_ = observer->update();
    }
  };

  imu_data_t getImuDataQueue()
  {
    return imu_data_queue_;
  }

private:
  imu_data_t imu_data_queue_;
};