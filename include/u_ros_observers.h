#ifndef U_ROS_OBSERVERS_H
#define U_ROS_OBSERVERS_H

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
class EventObserverInterface
{
public:
  virtual ~EventObserverInterface() {}
  friend class ImuRosEvent;
  friend class OdomRosEvent;

protected:
  template <typename Event>
  void update(Event &eventObj)
  {}
};


// 
class RosEventInterface
{
public:
  virtual ~RosEventInterface(){};
  virtual void add(EventObserverInterface* obsv) = 0;
  virtual void notify() = 0;

  template <typename T>
  void update(T* obsv)
  {}
};

class ImuRosEvent : public RosEventInterface
{
public:
  ~ImuRosEvent() {}

  void add(EventObserverInterface* obsv) override
  {
    _obsvs.push_back(obsv);
  };

  void notify() override
  {
    if (_obsvs.empty()) return;
    std::list<EventObserverInterface*>::iterator it = _obsvs.begin();
    while (it != _obsvs.end()) {
      (*it)->update(*this);  // get imu data
      it++;
    }
  };
  
  void setImuDataQueue(const imu_data_t &imu_data) { imu_data_queue_ = imu_data; };

private:
  std::list<EventObserverInterface*> _obsvs;
  imu_data_t imu_data_queue_;
};

class OdomRosEvent : public RosEventInterface
{
public:
  void add(EventObserverInterface* obsv) override
  {
    _obsvs.push_back(obsv);
  };

  void notify() override
  {
    if (_obsvs.empty()) return;
    std::list<EventObserverInterface*>::iterator it = _obsvs.begin();
    while (it != _obsvs.end()) {
      (*it)->update(*this);  // get imu data
      it++;
    }
  };

private:
  std::list<EventObserverInterface*> _obsvs;
  odom_data_t odom_data_;
};

#endif