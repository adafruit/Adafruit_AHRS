#ifndef __ADAFRUIT_10DOF_H__
#define __ADAFRUIT_10DOF_H__

#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_BMP085_U.h>
#include "Adafruit_Sensor_Set.h"

// 10DOF board with an accelerometer, magnetometer, gyroscope, and pressure sensor.
class Adafruit_10DOF: public Adafruit_Sensor_Set
{
public:
  Adafruit_10DOF();
  bool begin();
  Adafruit_Sensor* getSensor(sensors_type_t type);
  Adafruit_LSM303_Accel_Unified& getAccel() { return _accel; }
  Adafruit_LSM303_Mag_Unified& getMag() { return _mag; }
  Adafruit_L3GD20_Unified& getGyro() { return _gyro; }
  Adafruit_BMP085_Unified& getBMP() { return _bmp; }
  
private:
  Adafruit_LSM303_Accel_Unified _accel;
  Adafruit_LSM303_Mag_Unified _mag;
  Adafruit_L3GD20_Unified _gyro;
  Adafruit_BMP085_Unified _bmp;

};

#endif