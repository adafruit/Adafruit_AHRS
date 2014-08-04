#include "Adafruit_10DOF.h"

Adafruit_10DOF::Adafruit_10DOF():
  _accel(30301),
  _mag(30302),
  _gyro(30303),
  _bmp(18001)
{}

bool Adafruit_10DOF::begin() {
  if (!_accel.begin()) {
    return false;
  }
  if (!_mag.begin()) {
    return false;
  }
  if (!_gyro.begin()) {
    return false;
  }
  if (!_bmp.begin()) {
    return false;
  }
  return true;
}

Adafruit_Sensor* Adafruit_10DOF::getSensor(sensors_type_t type) {
  switch (type) {
    case SENSOR_TYPE_ACCELEROMETER:
      return &_accel;
    case SENSOR_TYPE_MAGNETIC_FIELD:
      return &_mag;
    case SENSOR_TYPE_GYROSCOPE:
      return &_gyro;
    case SENSOR_TYPE_PRESSURE:
      return &_bmp;
    default:
      return NULL;
  }
}
