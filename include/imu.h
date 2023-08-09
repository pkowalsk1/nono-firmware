#pragma once
#include <Adafruit_BNO055.h>
#include <Adafruit_SPIDevice.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>

#include "uros/observers.h"

class ImuDriver : public EventObserverInterface<imu_data_t>
{
public:
  ImuDriver(int32_t sensorID, uint8_t address) : id{sensorID}, addr{address} {};
  ~ImuDriver() {}

  bool init();

protected:
  void update(imu_data_t& data_queue) override
  {
    data_queue = queryImuData();
  }

private:
  imu_data_t queryImuData();

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
};