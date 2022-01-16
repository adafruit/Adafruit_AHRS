/*!
 * @file Adafruit_AHRS_NXPFusion.h
 *
 * @section license License
 *
 * This is a modification of
 * https://github.com/memsindustrygroup/Open-Source-Sensor-Fusion/blob/master/Sources/tasks.h
 * by PJRC / Paul Stoffregen https://github.com/PaulStoffregen/NXPMotionSense
 *
 * Copyright (c) 2014, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL FREESCALE SEMICONDUCTOR, INC. BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __Adafruit_Nxp_Fusion_h_
#define __Adafruit_Nxp_Fusion_h_

#include "Adafruit_AHRS_FusionInterface.h"
#include <Arduino.h>

/*!
 * @brief Kalman/NXP Fusion algorithm.
 */
class Adafruit_NXPSensorFusion : public Adafruit_AHRS_FusionInterface {
public:
  /**************************************************************************/
  /*!
   * @brief Initializes the 9DOF Kalman filter.
   *
   * @param sampleFrequency The sensor sample rate in herz(samples per second).
   */
  /**************************************************************************/
  void begin(float sampleFrequency = 100.0f);

  /**************************************************************************/
  /*!
   * @brief Updates the filter with new gyroscope, accelerometer, and
   * magnetometer data. For roll, pitch, and yaw the accelerometer values can be
   * either m/s^2 or g, but for linear acceleration they have to be in g.
   *
   * 9DOF orientation function implemented using a 12 element Kalman filter
   *
   * void fRun_9DOF_GBY_KALMAN(SV_9DOF_GBY_KALMAN_t *SV,
   * const AccelSensor_t *Accel, const MagSensor_t *Mag,
   * const GyroSensor_t *Gyro, const MagCalibration_t *MagCal)
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
  void update(float gx, float gy, float gz, float ax, float ay, float az,
              float mx, float my, float mz);

  float getRoll() { return PhiPl; }
  float getPitch() { return ThePl; }
  float getYaw() { return PsiPl; }

  void getQuaternion(float *w, float *x, float *y, float *z) {
    *w = qPl.q0;
    *x = qPl.q1;
    *y = qPl.q2;
    *z = qPl.q3;
  }

  void setQuaternion(float w, float x, float y, float z) {
    qPl.q0 = w;
    qPl.q1 = x;
    qPl.q2 = y;
    qPl.q3 = z;
  }
  /**************************************************************************/
  /*!
   * @brief Get the linear acceleration part of the acceleration value given to
   * update.
   *
   * @param x The pointer to write the linear acceleration x axis to. In g.
   * @param y The pointer to write the linear acceleration y axis to. In g.
   * @param z The pointer to write the linear acceleration z axis to. In g.
   */
  /**************************************************************************/
  void getLinearAcceleration(float *x, float *y, float *z) const {
    *x = aSePl[0];
    *y = aSePl[1];
    *z = aSePl[2];
  }

  /**************************************************************************/
  /*!
   * @brief Get the gravity vector from the gyroscope values.
   *
   * @param x A float pointer to write the gravity vector x component to. In g.
   * @param y A float pointer to write the gravity vector y component to. In g.
   * @param z A float pointer to write the gravity vector z component to. In g.
   */
  /**************************************************************************/
  void getGravityVector(float *x, float *y, float *z) {
    *x = gSeGyMi[0];
    *y = gSeGyMi[1];
    *z = gSeGyMi[2];
  }

  /**************************************************************************/
  /*!
   * @brief Get the geomagnetic vector in global frame.
   *
   * @param x The pointer to write the geomagnetic vector x axis to. In uT.
   * @param y The pointer to write the geomagnetic vector y axis to. In uT.
   * @param z The pointer to write the geomagnetic vector z axis to. In uT.
   */
  /**************************************************************************/
  void getGeomagneticVector(float *x, float *y, float *z) const {
    *x = mGl[0];
    *y = mGl[1];
    *z = mGl[2];
  }

  typedef struct {
    float q0; // w
    float q1; // x
    float q2; // y
    float q3; // z
  } Quaternion_t;

  // These are Madgwick & Mahony - extrinsic rotation reference (wrong!)
  // float getPitch() {return atan2f(2.0f * qPl.q2 * qPl.q3 - 2.0f * qPl.q0 *
  // qPl.q1, 2.0f * qPl.q0 * qPl.q0 + 2.0f * qPl.q3 * qPl.q3 - 1.0f);}; float
  // getRoll() {return -1.0f * asinf(2.0f * qPl.q1 * qPl.q3 + 2.0f * qPl.q0 *
  // qPl.q2);}; float getYaw() {return atan2f(2.0f * qPl.q1 * qPl.q2 - 2.0f *
  // qPl.q0 * qPl.q3, 2.0f * qPl.q0 * qPl.q0 + 2.0f * qPl.q1 * qPl.q1 - 1.0f);};

private:
  float PhiPl; // roll (deg)
  float ThePl; // pitch (deg)
  float PsiPl; // yaw (deg)
  float RhoPl; // compass (deg)
  float ChiPl; // tilt from vertical (deg)
  // orientation matrix, quaternion and rotation vector
  float RPl[3][3];  // a posteriori orientation matrix
  Quaternion_t qPl; // a posteriori orientation quaternion
  float RVecPl[3];  // rotation vector
  // angular velocity
  float Omega[3]; // angular velocity (deg/s)
  // systick timer for benchmarking
  int32_t systick; // systick timer;
  // end: elements common to all motion state vectors

  // elements transmitted over bluetooth in kalman packet
  float bPl[3];     // gyro offset (deg/s)
  float ThErrPl[3]; // orientation error (deg)
  float bErrPl[3];  // gyro offset error (deg/s)
  // end elements transmitted in kalman packet

  float dErrGlPl[3]; // magnetic disturbance error (uT, global frame)
  float dErrSePl[3]; // magnetic disturbance error (uT, sensor frame)
  float aErrSePl[3]; // linear acceleration error (g, sensor frame)
  float aSeMi[3];    // linear acceleration (g, sensor frame)
  float DeltaPl;     // inclination angle (deg)
  float aSePl[3];    // linear acceleration (g, sensor frame)
  float aGlPl[3];    // linear acceleration (g, global frame)
  float gErrSeMi[3]; // difference (g, sensor frame) of gravity vector (accel)
                     // and gravity vector (gyro)
  float mErrSeMi[3]; // difference (uT, sensor frame) of geomagnetic vector
                     // (magnetometer) and geomagnetic vector (gyro)
  float gSeGyMi[3];  // gravity vector (g, sensor frame) measurement from gyro
  float
      mSeGyMi[3]; // geomagnetic vector (uT, sensor frame) measurement from gyro
  float mGl[3];   // geomagnetic vector (uT, global frame)
  float QvAA;     // accelerometer terms of Qv
  float QvMM;     // magnetometer terms of Qv
  float PPlus12x12[12][12]; // covariance matrix P+
  float K12x6[12][6];       // kalman filter gain matrix K
  float Qw12x12[12][12];    // covariance matrix Qw
  float C6x12[6][12];       // measurement matrix C
  float RMi[3][3];          // a priori orientation matrix
  Quaternion_t Deltaq;      // delta quaternion
  Quaternion_t qMi;         // a priori orientation quaternion
  float casq;               // FCA * FCA;
  float cdsq;               // FCD * FCD;
  float Fastdeltat;         // sensor sampling interval (s) = 1 / SENSORFS
  float deltat;     // kalman filter sampling interval (s) = OVERSAMPLE_RATIO /
                    // SENSORFS
  float deltatsq;   // fdeltat * fdeltat
  float QwbplusQvG; // FQWB + FQVG
  int8_t
      FirstOrientationLock; // denotes that 9DOF orientation has locked to 6DOF
  int8_t resetflag;         // flag to request re-initialization on next pass
};

#endif
