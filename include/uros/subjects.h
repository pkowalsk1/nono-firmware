#pragma once
#include <vector>

#include "uros/observers.h"

template <typename DataQueueType>
class RosEventInterface
{
public:
  virtual ~RosEventInterface(){};

  void add(EventObserverInterface<DataQueueType>* obsv)
  {
    obsvs_.push_back(obsv);
    data_queue_.resize(obsvs_.size());
  }
  virtual void notify() = 0;

  std::vector<DataQueueType> getDataQueue()
  {
    return data_queue_;
  }
  void setDataQueue(const std::vector<DataQueueType>& data_queue)
  {
    data_queue_ = data_queue;
  }

protected:
  std::vector<EventObserverInterface<DataQueueType>*> obsvs_;
  std::vector<DataQueueType> data_queue_;
};

class ImuRosEvent : public RosEventInterface<imu_data_t>
{
public:
  void notify() override
  {
    if (obsvs_.empty()) return;
    for (size_t i = 0; i < obsvs_.size(); i++) {
      obsvs_[i]->update(data_queue_[i]);
    }
  }
};

class JointPubRosEvent : public RosEventInterface<joint_states_data_t>
{
public:
  void notify() override
  {
    if (obsvs_.empty()) return;
    for (size_t i = 0; i < obsvs_.size(); i++) {
      obsvs_[i]->update(data_queue_[i]);
    }
  }
};

class MotorsCmdRosEvent : public RosEventInterface<motors_cmd_data_t>
{
public:
  void notify() override
  {
    if (obsvs_.empty()) return;
    for (size_t i = 0; i < obsvs_.size(); i++) {
      obsvs_[i]->update(data_queue_[i]);
    }
  }
};