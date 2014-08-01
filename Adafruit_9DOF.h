#ifndef __ADAFRUIT_9DOF_H__
#define __ADAFRUIT_9DOF_H__

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include "Adafruit_Sensor_Set.h"

// 9DOF board with an accelerometer, magnetometer, and gyroscope.
class Adafruit_9DOF: public Adafruit_Sensor_Set
{
public:
  Adafruit_9DOF();
  bool begin();
  Adafruit_Sensor* getSensor(sensors_type_t type);
  Adafruit_LSM303_Accel_Unified& getAccel() { return _accel; }
  Adafruit_LSM303_Mag_Unified& getMag() { return _mag; }
  Adafruit_L3GD20_Unified& getGyro() { return _gyro; }
  
private:
  Adafruit_LSM303_Accel_Unified _accel;
  Adafruit_LSM303_Mag_Unified _mag;
  Adafruit_L3GD20_Unified _gyro;

};

#endif