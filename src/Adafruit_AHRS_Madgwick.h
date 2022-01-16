//=============================================================================================
// Madgwick.h
//=============================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// From the x-io website "Open-source resources available on this website are
// provided under the GNU General Public Licence unless an alternative licence
// is provided in source."
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=============================================================================================
#ifndef __Adafruit_Madgwick_h__
#define __Adafruit_Madgwick_h__

#include "Adafruit_AHRS_FusionInterface.h"
#include <math.h>

//--------------------------------------------------------------------------------------------
// Variable declaration
class Adafruit_Madgwick : public Adafruit_AHRS_FusionInterface {
private:
  static float invSqrt(float x);
  float beta; // algorithm gain
  float q0;
  float q1;
  float q2;
  float q3; // quaternion of sensor frame relative to auxiliary frame
  float invSampleFreq;
  float roll, pitch, yaw;
  float grav[3];
  bool anglesComputed = false;
  void computeAngles();

  //-------------------------------------------------------------------------------------------
  // Function declarations
public:
  Adafruit_Madgwick();
  Adafruit_Madgwick(float gain);
  void begin(float sampleFrequency) { invSampleFreq = 1.0f / sampleFrequency; }
  void update(float gx, float gy, float gz, float ax, float ay, float az,
              float mx, float my, float mz);
  void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);
  void update(float gx, float gy, float gz, float ax, float ay, float az,
              float mx, float my, float mz, float dt);
  void updateIMU(float gx, float gy, float gz, float ax, float ay, float az,
                 float dt);
  // float getPitch(){return atan2f(2.0f * q2 * q3 - 2.0f * q0 * q1, 2.0f * q0 *
  // q0 + 2.0f * q3 * q3 - 1.0f);}; float getRoll(){return -1.0f * asinf(2.0f *
  // q1 * q3 + 2.0f * q0 * q2);}; float getYaw(){return atan2f(2.0f * q1 * q2
  // - 2.0f * q0 * q3, 2.0f * q0 * q0 + 2.0f * q1 * q1 - 1.0f);};
  float getBeta() { return beta; }
  void setBeta(float beta) { this->beta = beta; }
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
