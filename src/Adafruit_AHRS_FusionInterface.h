/*!
 * @file Adafruit_AHRS_FusionInterface.h
 *
 * @section license License
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Ha Thach (tinyusb.org) for Adafruit Industries
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef ADAFRUIT_AHRS_FUSIONINTERFACE_H_
#define ADAFRUIT_AHRS_FUSIONINTERFACE_H_

/*!
 * @brief The common interface for the fusion algorithms.
 */
class Adafruit_AHRS_FusionInterface {
public:
  /**************************************************************************/
  /*!
   * @brief Initializes the sensor fusion filter.
   *
   * @param sampleFrequency The sensor sample rate in herz(samples per second).
   */
  /**************************************************************************/
  virtual void begin(float sampleFrequency) = 0;

  /**************************************************************************/
  /*!
   * @brief Updates the filter with new gyroscope, accelerometer, and
   * magnetometer data.
   *
   * @param gx The gyroscope x axis. In DPS.
   * @param gy The gyroscope y axis. In DPS.
   * @param gz The gyroscope z axis. In DPS.
   * @param ax The accelerometer x axis. In g.
   * @param ay The accelerometer y axis. In g.
   * @param az The accelerometer z axis. In g.
   * @param mx The magnetometer x axis. In uT.
   * @param my The magnetometer y axis. In uT.
   * @param mz The magnetometer z axis. In uT.
   */
  /**************************************************************************/
  virtual void update(float gx, float gy, float gz, float ax, float ay,
                      float az, float mx, float my, float mz) = 0;

  /**************************************************************************/
  /*!
   * @brief Gets the current roll of the sensors.
   *
   * @return The current sensor roll.
   */
  /**************************************************************************/
  virtual float getRoll() = 0;

  /**************************************************************************/
  /*!
   * @brief Gets the current pitch of the sensors.
   *
   * @return The current sensor pitch.
   */
  /**************************************************************************/
  virtual float getPitch() = 0;

  /**************************************************************************/
  /*!
   * @brief Gets the current yaw of the sensors.
   *
   * @return The current sensor yaw.
   */
  /**************************************************************************/
  virtual float getYaw() = 0;
  virtual void getQuaternion(float *w, float *x, float *y, float *z) = 0;
  virtual void setQuaternion(float w, float x, float y, float z) = 0;

  /**************************************************************************/
  /*!
   * @brief Gets the current gravity vector of the sensor.
   *
   * @param x A float pointer to write the gravity vector x component to. In g.
   * @param y A float pointer to write the gravity vector y component to. In g.
   * @param z A float pointer to write the gravity vector z component to. In g.
   */
  virtual void getGravityVector(float *x, float *y, float *z) = 0;
};

#endif /* ADAFRUIT_AHRS_FUSIONINTERFACE_H_ */
