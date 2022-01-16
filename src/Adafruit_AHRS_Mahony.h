//=============================================================================================
// Adafruit_AHRS_Mahony.h
//=============================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=============================================================================================
#ifndef __Adafruit_Mahony_h__
#define __Adafruit_Mahony_h__

#include "Adafruit_AHRS_FusionInterface.h"
#include <math.h>

//--------------------------------------------------------------------------------------------
// Variable declaration

class Adafruit_Mahony : public Adafruit_AHRS_FusionInterface {
private:
  float twoKp; // 2 * proportional gain (Kp)
  float twoKi; // 2 * integral gain (Ki)
  float q0, q1, q2,
      q3; // quaternion of sensor frame relative to auxiliary frame
  float integralFBx, integralFBy,
      integralFBz; // integral error terms scaled by Ki
  float invSampleFreq;
  float roll, pitch, yaw;
  float grav[3];
  bool anglesComputed = false;
  static float invSqrt(float x);
  void computeAngles();

  //-------------------------------------------------------------------------------------------
  // Function declarations

public:
  Adafruit_Mahony();
  Adafruit_Mahony(float prop_gain, float int_gain);
  void begin(float sampleFrequency) { invSampleFreq = 1.0f / sampleFrequency; }
  void update(float gx, float gy, float gz, float ax, float ay, float az,
              float mx, float my, float mz);
  void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);
  void update(float gx, float gy, float gz, float ax, float ay, float az,
              float mx, float my, float mz, float dt);
  void updateIMU(float gx, float gy, float gz, float ax, float ay, float az,
                 float dt);
  float getKp() { return twoKp / 2.0f; }
  void setKp(float Kp) { twoKp = 2.0f * Kp; }
  float getKi() { return twoKi / 2.0f; }
  void setKi(float Ki) { twoKi = 2.0f * Ki; }
  float getRoll() {
    if (!anglesComputed)
      computeAngles();
    return roll * 57.29578f;
  }
  float getPitch() {
    if (!anglesComputed)
      computeAngles();
    return pitch * 57.29578f;
  }
  float getYaw() {
    if (!anglesComputed)
      computeAngles();
    return yaw * 57.29578f + 180.0f;
  }
  float getRollRadians() {
    if (!anglesComputed)
      computeAngles();
    return roll;
  }
  float getPitchRadians() {
    if (!anglesComputed)
      computeAngles();
    return pitch;
  }
  float getYawRadians() {
    if (!anglesComputed)
      computeAngles();
    return yaw;
  }
  void getQuaternion(float *w, float *x, float *y, float *z) {
    *w = q0;
    *x = q1;
    *y = q2;
    *z = q3;
  }
  void setQuaternion(float w, float x, float y, float z) {
    q0 = w;
    q1 = x;
    q2 = y;
    q3 = z;
  }
  void getGravityVector(float *x, float *y, float *z) {
    if (!anglesComputed)
      computeAngles();
    *x = grav[0];
    *y = grav[1];
    *z = grav[2];
  }
};

#endif
