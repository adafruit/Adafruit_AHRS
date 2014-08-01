#include "Adafruit_9DOF.h"

Adafruit_9DOF::Adafruit_9DOF():
  _accel(30301),
  _mag(30302),
  _gyro(30303)
{}

bool Adafruit_9DOF::begin() {
  if (!_accel.begin()) {
    return false;
  }
  if (!_mag.begin()) {
    return false;
  }
  if (!_gyro.begin()) {
    return false;
  }
}

Adafruit_Sensor* Adafruit_9DOF::getSensor(sensors_type_t type) {
  switch (type) {
    case SENSOR_TYPE_ACCELEROMETER:
      return &_accel;
    case SENSOR_TYPE_MAGNETIC_FIELD:
      return &_mag;
    case SENSOR_TYPE_GYROSCOPE:
      return &_gyro;
    default:
      return NULL;
  }
}