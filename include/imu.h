#ifndef IMU_H
#define IMU_H

#include <Adafruit_BNO055.h>
#include <Adafruit_SPIDevice.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>

#include "u_ros_observers.h"

class ImuDriver : EventObserverInterface
{
public:
  ImuDriver(int32_t sensorID, uint8_t address) : id{sensorID}, addr{address} {};
  ~ImuDriver() {}

  bool init();                // TODO: update for obsv
  imu_data_t get_imu_data();  // TODO: update for obsv

protected:
  void update(ImuRosEvent& event)
  {
    event.setImuDataQueue(my_data_);
  }

private:
  int32_t id;
  uint8_t addr;

  Adafruit_BNO055 bno = Adafruit_BNO055(id, addr);

  sensors_event_t orientation_data_;
  sensors_event_t ang_velocity_data_;
  sensors_event_t linear_accel_data_;
  sensors_event_t magnetometer_data_;
  sensors_event_t accelerometer_data_;
  sensors_event_t gravity_data_;

  imu::Quaternion quat;

  // TODO: temp
  imu_data_t my_data_;
};

#endif /* IMU_H */