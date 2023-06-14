#ifndef U_ROS_SUBJECTS_H
#define U_ROS_SUBJECTS_H

#include <list>
#include "u_ros_observers.h"

class RosEventInterface
{
public:
  virtual ~RosEventInterface(){};
  virtual void add(EventObserverInterface* obsv) = 0;
  virtual void notify() = 0;
};

class RosEvent : public RosEventInterface
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
      (*it)->update();  // get imu data
      it++;
    }
  };

private:
  std::list<EventObserverInterface*> _obsvs;
};

#endif