#include "imu.h"

bool ImuDriver::init()
{
  while (!bno.begin()) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(400);
  }

  return true;
}

imu_data_t ImuDriver::queryImuData()
{
  bno.getEvent(&ang_velocity_data_, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linear_accel_data_, Adafruit_BNO055::VECTOR_LINEARACCEL);
  quat = bno.getQuat();

  imu_data_t bno_data;
  bno_data.orientation[0] = quat.x();
  bno_data.orientation[1] = quat.y();
  bno_data.orientation[2] = quat.z();
  bno_data.orientation[3] = quat.w();

  sensors_event_t* gyro = &ang_velocity_data_;
  bno_data.angular_velocity[0] = gyro->gyro.x;
  bno_data.angular_velocity[1] = gyro->gyro.y;
  bno_data.angular_velocity[2] = gyro->gyro.z;

  sensors_event_t* acceleration = &linear_accel_data_;
  bno_data.linear_acceleration[0] = acceleration->acceleration.x;
  bno_data.linear_acceleration[1] = acceleration->acceleration.y;
  bno_data.linear_acceleration[2] = acceleration->acceleration.z;

  return bno_data;
}
