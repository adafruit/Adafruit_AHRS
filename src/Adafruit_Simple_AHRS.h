/*!
 * @file Adafruit_Simple_AHRS.cpp
 */
#ifndef __ADAFRUIT_SIMPLE_AHRS_H__
#define __ADAFRUIT_SIMPLE_AHRS_H__

#include "Adafruit_Sensor_Set.h"
#include <Adafruit_Sensor.h>

/*!
 * @brief Simple sensor fusion AHRS using an accelerometer and magnetometer.
 */
class Adafruit_Simple_AHRS {
public:
  /**************************************************************************/
  /*!
   * @brief Create a simple AHRS from a device with multiple sensors.
   *
   * @param accelerometer The accelerometer to use for this sensor fusion.
   * @param magnetometer The magnetometer to use for this sensor fusion.
   */
  /**************************************************************************/
  Adafruit_Simple_AHRS(Adafruit_Sensor *accelerometer,
                       Adafruit_Sensor *magnetometer);

  /**************************************************************************/
  /*!
   * @brief Create a simple AHRS from a device with multiple sensors.
   *
   * @param sensors A set of sensors containing the accelerometer and
   * magnetometer for this sensor fusion.
   */
  /**************************************************************************/
  Adafruit_Simple_AHRS(Adafruit_Sensor_Set &sensors);

  /**************************************************************************/
  /*!
   * @brief Compute orientation based on accelerometer and magnetometer data.
   *
   * @return Whether the orientation was computed.
   */
  /**************************************************************************/
  bool getOrientation(sensors_vec_t *orientation);

private:
  Adafruit_Sensor *_accel;
  Adafruit_Sensor *_mag;
};

#endif
