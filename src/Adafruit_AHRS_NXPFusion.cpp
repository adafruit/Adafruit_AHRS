/*!
 * @file Adafruit_AHRS_NXPFusion.cpp
 *
 * @section license License
 *
 * Copyright (c) 2014, Freescale Semiconductor, Inc.
 * All rights reserved.
 * vim: set ts=4:
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
 *
 * This is the file that contains the fusion routines.  It is STRONGLY
 * RECOMMENDED that the casual developer NOT TOUCH THIS FILE.  The mathematics
 * behind this file is extremely complex, and it will be very easy (almost
 * inevitable) that you screw it up.
 */

#include "Adafruit_AHRS_NXPFusion.h"

// kalman filter noise variances
#define FQVA_9DOF_GBY_KALMAN 2E-6F // accelerometer noise g^2 so 1.4mg RMS
#define FQVM_9DOF_GBY_KALMAN 0.1F  // magnetometer noise uT^2
#define FQVG_9DOF_GBY_KALMAN 0.3F  // gyro noise (deg/s)^2
#define FQWB_9DOF_GBY_KALMAN                                                   \
  1E-9F // gyro offset drift (deg/s)^2: 1E-9 implies 0.09deg/s max at 50Hz
#define FQWA_9DOF_GBY_KALMAN                                                   \
  1E-4F // linear acceleration drift g^2 (increase slows convergence to g but
        // reduces sensitivity to shake)
#define FQWD_9DOF_GBY_KALMAN                                                   \
  0.5F // magnetic disturbance drift uT^2 (increase slows convergence to B but
       // reduces sensitivity to magnet)
// initialization of Qw covariance matrix
#define FQWINITTHTH_9DOF_GBY_KALMAN 2000E-5F // th_e * th_e terms
#define FQWINITBB_9DOF_GBY_KALMAN 250E-3F    // b_e * b_e terms
#define FQWINITTHB_9DOF_GBY_KALMAN 0.0F      // th_e * b_e terms
#define FQWINITAA_9DOF_GBY_KALMAN                                              \
  10E-5F // a_e * a_e terms (increase slows convergence to g but reduces
         // sensitivity to shake)
#define FQWINITDD_9DOF_GBY_KALMAN                                              \
  600E-3F // d_e * d_e terms (increase slows convergence to B but reduces
          // sensitivity to magnet)
// linear acceleration and magnetic disturbance time constants
#define FCA_9DOF_GBY_KALMAN 0.5F // linear acceleration decay factor
#define FCD_9DOF_GBY_KALMAN 0.5F // magnetic disturbance decay factor
// maximum geomagnetic inclination angle tracked by Kalman filter
#define SINDELTAMAX                                                            \
  0.9063078F // sin of max +ve geomagnetic inclination angle: here 65.0 deg
#define COSDELTAMAX                                                            \
  0.4226183F // cos of max +ve geomagnetic inclination angle: here 65.0 deg
#define DEFAULTB 50.0F // default geomagnetic field (uT)
#define X 0            // vector components
#define Y 1
#define Z 2
#define FDEGTORAD 0.01745329251994F  // degrees to radians conversion = pi / 180
#define FRADTODEG 57.2957795130823F  // radians to degrees conversion = 180 / pi
#define ONEOVER48 0.02083333333F     // 1 / 48
#define ONEOVER3840 0.0002604166667F // 1 / 3840

#define Quaternion_t Adafruit_NXPSensorFusion::Quaternion_t

static void fqAeq1(Quaternion_t *pqA);
static void feCompassNED(float fR[][3], float *pfDelta, const float fBc[],
                         const float fGp[]);
static void fNEDAnglesDegFromRotationMatrix(float R[][3], float *pfPhiDeg,
                                            float *pfTheDeg, float *pfPsiDeg,
                                            float *pfRhoDeg, float *pfChiDeg);
static void fQuaternionFromRotationMatrix(float R[][3], Quaternion_t *pq);
static void fQuaternionFromRotationVectorDeg(Quaternion_t *pq,
                                             const float rvecdeg[],
                                             float fscaling);
static void fRotationMatrixFromQuaternion(float R[][3], const Quaternion_t *pq);
static void fRotationVectorDegFromQuaternion(Quaternion_t *pq, float rvecdeg[]);
static void qAeqAxB(Quaternion_t *pqA, const Quaternion_t *pqB);
static void qAeqBxC(Quaternion_t *pqA, const Quaternion_t *pqB,
                    const Quaternion_t *pqC);
// static Quaternion_t qconjgAxB(const Quaternion_t *pqA, const Quaternion_t
// *pqB);
static void fqAeqNormqA(Quaternion_t *pqA);
static float fasin_deg(float x);
static float facos_deg(float x);
static float fatan_deg(float x);
static float fatan2_deg(float y, float x);
static float fatan_15deg(float x);

extern "C" {
void f3x3matrixAeqI(float A[][3]);
void fmatrixAeqI(float *A[], int16_t rc);
void f3x3matrixAeqScalar(float A[][3], float Scalar);
void f3x3matrixAeqInvSymB(float A[][3], float B[][3]);
void f3x3matrixAeqAxScalar(float A[][3], float Scalar);
void f3x3matrixAeqMinusA(float A[][3]);
float f3x3matrixDetA(float A[][3]);
void eigencompute(float A[][10], float eigval[], float eigvec[][10], int8_t n);
void fmatrixAeqInvA(float *A[], int8_t iColInd[], int8_t iRowInd[],
                    int8_t iPivot[], int8_t isize);
void fmatrixAeqRenormRotA(float A[][3]);
}

/**************************************************************************/
/*!
 * @brief Initializes the 9DOF Kalman filter.
 */
/**************************************************************************/
void Adafruit_NXPSensorFusion::begin(float sampleFrequency) {
  int8_t i, j;

  // reset the flag denoting that a first 9DOF orientation lock has been
  // achieved
  FirstOrientationLock = 0;

  // compute and store useful product terms to save floating point calculations
  // later
  Fastdeltat = 1.0f / sampleFrequency;
  deltat = Fastdeltat;
  deltatsq = deltat * deltat;
  casq = FCA_9DOF_GBY_KALMAN * FCA_9DOF_GBY_KALMAN;
  cdsq = FCD_9DOF_GBY_KALMAN * FCD_9DOF_GBY_KALMAN;
  QwbplusQvG = FQWB_9DOF_GBY_KALMAN + FQVG_9DOF_GBY_KALMAN;

  // initialize the fixed entries in the measurement matrix C
  for (i = 0; i < 6; i++) {
    for (j = 0; j < 12; j++) {
      C6x12[i][j] = 0.0F;
    }
  }
  C6x12[0][6] = C6x12[1][7] = C6x12[2][8] = 1.0F;
  C6x12[3][9] = C6x12[4][10] = C6x12[5][11] = -1.0F;

  // zero a posteriori orientation, error vector xe+ (thetae+, be+, de+, ae+)
  // and b+ and inertial
  f3x3matrixAeqI(RPl);
  fqAeq1(&qPl);
  for (i = X; i <= Z; i++) {
    ThErrPl[i] = bErrPl[i] = aErrSePl[i] = dErrSePl[i] = bPl[i] = 0.0F;
  }

  // initialize the reference geomagnetic vector (uT, global frame)
  DeltaPl = 0.0F;
  // initialize NED geomagnetic vector to zero degrees inclination
  mGl[X] = DEFAULTB;
  mGl[Y] = 0.0F;
  mGl[Z] = 0.0F;

  // initialize noise variances for Qv and Qw matrix updates
  QvAA = FQVA_9DOF_GBY_KALMAN + FQWA_9DOF_GBY_KALMAN +
         FDEGTORAD * FDEGTORAD * deltatsq *
             (FQWB_9DOF_GBY_KALMAN + FQVG_9DOF_GBY_KALMAN);
  QvMM = FQVM_9DOF_GBY_KALMAN + FQWD_9DOF_GBY_KALMAN +
         FDEGTORAD * FDEGTORAD * deltatsq * DEFAULTB * DEFAULTB *
             (FQWB_9DOF_GBY_KALMAN + FQVG_9DOF_GBY_KALMAN);

  // initialize the 12x12 noise covariance matrix Qw of the a priori error
  // vector xe- Qw is then recursively updated as P+ = (1 - K * C) * P- = (1 - K
  // * C) * Qw  and Qw updated from P+ zero the matrix Qw
  for (i = 0; i < 12; i++) {
    for (j = 0; j < 12; j++) {
      Qw12x12[i][j] = 0.0F;
    }
  }
  // loop over non-zero values
  for (i = 0; i < 3; i++) {
    // theta_e * theta_e terms
    Qw12x12[i][i] = FQWINITTHTH_9DOF_GBY_KALMAN;
    // b_e * b_e terms
    Qw12x12[i + 3][i + 3] = FQWINITBB_9DOF_GBY_KALMAN;
    // th_e * b_e terms
    Qw12x12[i][i + 3] = Qw12x12[i + 3][i] = FQWINITTHB_9DOF_GBY_KALMAN;
    // a_e * a_e terms
    Qw12x12[i + 6][i + 6] = FQWINITAA_9DOF_GBY_KALMAN;
    // d_e * d_e terms
    Qw12x12[i + 9][i + 9] = FQWINITDD_9DOF_GBY_KALMAN;
  }

  // clear the reset flag
  resetflag = 0;
}

/**************************************************************************/
/*!
 * @brief Updates the filter with new gyroscope, accelerometer, and magnetometer
 * data.
 */
/**************************************************************************/
void Adafruit_NXPSensorFusion::update(float gx, float gy, float gz, float ax,
                                      float ay, float az, float mx, float my,
                                      float mz) {
  float Accel[3] = {ax, ay, az}; // Accel
  float Yp[3] = {gx, gy, gz};    // Gryo
  float Mag[3] = {mx, my, mz};   // Mag

  // local scalars and arrays
  float fopp, fadj, fhyp;     // opposite, adjacent and hypoteneuse
  float fsindelta, fcosdelta; // sin and cos of inclination angle delta
  float rvec[3];              // rotation vector
  float ftmp;                 // scratch variable
  float ftmpA12x6[12][6];     // scratch array
  int8_t i, j, k;             // loop counters
  int8_t iMagJamming;         // magnetic jamming flag
  int8_t ValidMagCal;

  // assorted array pointers
  float *pfPPlus12x12ij;
  float *pfPPlus12x12kj;
  float *pfQw12x12ij;
  float *pfQw12x12ik;
  float *pfQw12x12kj;
  float *pfK12x6ij;
  float *pfK12x6ik;
  float *pftmpA12x6ik;
  float *pftmpA12x6kj;
  float *pftmpA12x6ij;
  float *pfC6x12ik;
  float *pfC6x12jk;

  // working arrays for 6x6 matrix inversion
  float *pfRows[6];
  int8_t iColInd[6];
  int8_t iRowInd[6];
  int8_t iPivot[6];

  // do a reset and return if requested
  if (resetflag) {
    begin(1.0f / deltat);
    return;
  }

  // *********************************************************************************
  // initial orientation lock to accelerometer and magnetometer eCompass
  // orientation
  // *********************************************************************************
  if (fabsf(mx) <= 50.0f && fabsf(my) <= 50.0f && fabsf(mz) <= 50.0f) {
    ValidMagCal = 1;
  } else {
    ValidMagCal = 0;
  }

  // do a once-only orientation lock after the first valid magnetic calibration
  if (ValidMagCal && !FirstOrientationLock) {
    // get the 6DOF orientation matrix and initial inclination angle
    feCompassNED(RPl, &DeltaPl, Mag, Accel);

    // get the orientation quaternion from the orientation matrix
    fQuaternionFromRotationMatrix(RPl, &qPl);

    // set the orientation lock flag so this initial alignment is only performed
    // once
    FirstOrientationLock = 1;
  }

  // *********************************************************************************
  // calculate a priori rotation matrix
  // *********************************************************************************

  // compute the angular velocity from the averaged high frequency gyro reading.
  // omega[k] = yG[k] - b-[k] = yG[k] - b+[k-1] (deg/s)
  Omega[X] = Yp[X] - bPl[X];
  Omega[Y] = Yp[Y] - bPl[Y];
  Omega[Z] = Yp[Z] - bPl[Z];

  // initialize the a priori orientation quaternion to the previous a posteriori
  // estimate
  qMi = qPl;

  // integrate the buffered high frequency (typically 200Hz) gyro readings
  // for (j = 0; j < OVERSAMPLE_RATIO; j++) {
  // compute the incremental fast (typically 200Hz) rotation vector rvec (deg)
  for (i = X; i <= Z; i++) {
    rvec[i] = (Yp[i] - bPl[i]) * Fastdeltat;
  }

  // compute the incremental quaternion fDeltaq from the rotation vector
  fQuaternionFromRotationVectorDeg(&Deltaq, rvec, 1.0F);

  // incrementally rotate the a priori orientation quaternion fqMi
  // the a posteriori quaternion fqPl is re-normalized later so this update is
  // stable
  qAeqAxB(&qMi, &Deltaq);
  //}

  // get the a priori rotation matrix from the a priori quaternion
  fRotationMatrixFromQuaternion(RMi, &qMi);

  // *********************************************************************************
  // calculate a priori gyro, accelerometer and magnetometer estimates
  // of the gravity and geomagnetic vectors and errors
  // the most recent 'Fast' measurements are used to reduce phase errors
  // *********************************************************************************

  for (i = X; i <= Z; i++) {
    // compute the a priori gyro estimate of the gravitational vector (g, sensor
    // frame) using an absolute rotation of the global frame gravity vector
    // (with magnitude 1g) NED gravity is along positive z axis
    gSeGyMi[i] = RMi[i][Z];

    // compute the a priori acceleration (a-) (g, sensor frame) from decayed a
    // posteriori estimate (g, sensor frame)
    aSeMi[i] = FCA_9DOF_GBY_KALMAN * aSePl[i];

    // compute the a priori gravity error vector (accelerometer minus gyro
    // estimates) (g, sensor frame) NED and Windows 8 have positive sign for
    // gravity: y = g - a and g = y + a
    gErrSeMi[i] = Accel[i] + aSeMi[i] - gSeGyMi[i];

    // compute the a priori gyro estimate of the geomagnetic vector (uT, sensor
    // frame) using an absolute rotation of the global frame geomagnetic vector
    // (with magnitude B uT) NED y component of geomagnetic vector in global
    // frame is zero
    mSeGyMi[i] = RMi[i][X] * mGl[X] + RMi[i][Z] * mGl[Z];

    // compute the a priori geomagnetic error vector (magnetometer minus gyro
    // estimates) (g, sensor frame)
    mErrSeMi[i] = Mag[i] - mSeGyMi[i];
  }

  // *********************************************************************************
  // update variable elements of measurement matrix C
  // *********************************************************************************

  // update measurement matrix C with -alpha(g-)x and -alpha(m-)x from gyro (g,
  // uT, sensor frame)
  C6x12[0][1] = FDEGTORAD * gSeGyMi[Z];
  C6x12[0][2] = -FDEGTORAD * gSeGyMi[Y];
  C6x12[1][2] = FDEGTORAD * gSeGyMi[X];
  C6x12[1][0] = -C6x12[0][1];
  C6x12[2][0] = -C6x12[0][2];
  C6x12[2][1] = -C6x12[1][2];
  C6x12[3][1] = FDEGTORAD * mSeGyMi[Z];
  C6x12[3][2] = -FDEGTORAD * mSeGyMi[Y];
  C6x12[4][2] = FDEGTORAD * mSeGyMi[X];
  C6x12[4][0] = -C6x12[3][1];
  C6x12[5][0] = -C6x12[3][2];
  C6x12[5][1] = -C6x12[4][2];
  C6x12[0][4] = -deltat * C6x12[0][1];
  C6x12[0][5] = -deltat * C6x12[0][2];
  C6x12[1][5] = -deltat * C6x12[1][2];
  C6x12[1][3] = -C6x12[0][4];
  C6x12[2][3] = -C6x12[0][5];
  C6x12[2][4] = -C6x12[1][5];
  C6x12[3][4] = -deltat * C6x12[3][1];
  C6x12[3][5] = -deltat * C6x12[3][2];
  C6x12[4][5] = -deltat * C6x12[4][2];
  C6x12[4][3] = -C6x12[3][4];
  C6x12[5][3] = -C6x12[3][5];
  C6x12[5][4] = -C6x12[4][5];

  // *********************************************************************************
  // calculate the Kalman gain matrix K
  // K = P- * C^T * inv(C * P- * C^T + Qv) = Qw * C^T * inv(C * Qw * C^T + Qv)
  // Qw is used as a proxy for P- throughout the code
  // P+ is used here as a working array to reduce RAM usage and is re-computed
  // later
  // *********************************************************************************

  // set ftmpA12x6 = P- * C^T = Qw * C^T where Qw and C are both sparse
  // C also has a significant number of +1 and -1 entries
  // ftmpA12x6 is also sparse but not symmetric
  for (i = 0; i < 12; i++) { // loop over rows of ftmpA12x6
    // initialize pftmpA12x6ij for current i, j=0
    pftmpA12x6ij = ftmpA12x6[i];

    for (j = 0; j < 6; j++) { // loop over columns of ftmpA12x6
      // zero ftmpA12x6[i][j]
      *pftmpA12x6ij = 0.0F;

      // initialize pfC6x12jk for current j, k=0
      pfC6x12jk = C6x12[j];

      // initialize pfQw12x12ik for current i, k=0
      pfQw12x12ik = Qw12x12[i];

      // sum matrix products over inner loop over k
      for (k = 0; k < 12; k++) {
        if ((*pfQw12x12ik != 0.0F) && (*pfC6x12jk != 0.0F)) {
          if (*pfC6x12jk == 1.0F)
            *pftmpA12x6ij += *pfQw12x12ik;
          else if (*pfC6x12jk == -1.0F)
            *pftmpA12x6ij -= *pfQw12x12ik;
          else
            *pftmpA12x6ij += *pfQw12x12ik * *pfC6x12jk;
        }

        // increment pfC6x12jk and pfQw12x12ik for next iteration of k
        pfC6x12jk++;
        pfQw12x12ik++;
      }

      // increment pftmpA12x6ij for next iteration of j
      pftmpA12x6ij++;
    }
  }

  // set symmetric P+ (6x6 scratch sub-matrix) to C * P- * C^T + Qv
  // = C * (Qw * C^T) + Qv = C * ftmpA12x6 + Qv
  // both C and ftmpA12x6 are sparse but not symmetric
  for (i = 0; i < 6; i++) { // loop over rows of P+
    // initialize pfPPlus12x12ij for current i, j=i
    pfPPlus12x12ij = PPlus12x12[i] + i;

    for (j = i; j < 6; j++) { // loop over above diagonal columns of P+
      // zero P+[i][j]
      *pfPPlus12x12ij = 0.0F;

      // initialize pfC6x12ik for current i, k=0
      pfC6x12ik = C6x12[i];

      // initialize pftmpA12x6kj for current j, k=0
      pftmpA12x6kj = *ftmpA12x6 + j;

      // sum matrix products over inner loop over k
      for (k = 0; k < 12; k++) {
        if ((*pfC6x12ik != 0.0F) && (*pftmpA12x6kj != 0.0F)) {
          if (*pfC6x12ik == 1.0F)
            *pfPPlus12x12ij += *pftmpA12x6kj;
          else if (*pfC6x12ik == -1.0F)
            *pfPPlus12x12ij -= *pftmpA12x6kj;
          else
            *pfPPlus12x12ij += *pfC6x12ik * *pftmpA12x6kj;
        }

        // update pfC6x12ik and pftmpA12x6kj for next iteration of k
        pfC6x12ik++;
        pftmpA12x6kj += 6;
      }

      // increment pfPPlus12x12ij for next iteration of j
      pfPPlus12x12ij++;
    }
  }

  // add in noise covariance terms to the diagonal
  PPlus12x12[0][0] += QvAA;
  PPlus12x12[1][1] += QvAA;
  PPlus12x12[2][2] += QvAA;
  PPlus12x12[3][3] += QvMM;
  PPlus12x12[4][4] += QvMM;
  PPlus12x12[5][5] += QvMM;

  // copy above diagonal elements of P+ (6x6 scratch sub-matrix) to below
  // diagonal
  for (i = 1; i < 6; i++)
    for (j = 0; j < i; j++)
      PPlus12x12[i][j] = PPlus12x12[j][i];

  // calculate inverse of P+ (6x6 scratch sub-matrix) = inv(C * P- * C^T + Qv) =
  // inv(C * Qw * C^T + Qv)
  for (i = 0; i < 6; i++) {
    pfRows[i] = PPlus12x12[i];
  }
  fmatrixAeqInvA(pfRows, iColInd, iRowInd, iPivot, 3);

  // set K = P- * C^T * inv(C * P- * C^T + Qv) = Qw * C^T * inv(C * Qw * C^T +
  // Qv) = ftmpA12x6 * P+ (6x6 sub-matrix) ftmpA12x6 = Qw * C^T is sparse but P+
  // (6x6 sub-matrix) is not K is not symmetric because C is not symmetric
  for (i = 0; i < 12; i++) { // loop over rows of K12x6
    // initialize pfK12x6ij for current i, j=0
    pfK12x6ij = K12x6[i];

    for (j = 0; j < 6; j++) { // loop over columns of K12x6
      // zero the matrix element fK12x6[i][j]
      *pfK12x6ij = 0.0F;

      // initialize pftmpA12x6ik for current i, k=0
      pftmpA12x6ik = ftmpA12x6[i];

      // initialize pfPPlus12x12kj for current j, k=0
      pfPPlus12x12kj = *PPlus12x12 + j;

      // sum matrix products over inner loop over k
      for (k = 0; k < 6; k++) {
        if (*pftmpA12x6ik != 0.0F) {
          *pfK12x6ij += *pftmpA12x6ik * *pfPPlus12x12kj;
        }

        // increment pftmpA12x6ik and pfPPlus12x12kj for next iteration of k
        pftmpA12x6ik++;
        pfPPlus12x12kj += 12;
      }

      // increment pfK12x6ij for the next iteration of j
      pfK12x6ij++;
    }
  }

  // *********************************************************************************
  // calculate a posteriori error estimate: xe+ = K * ze-
  // *********************************************************************************

  // first calculate all four error vector components using accelerometer error
  // component only for fThErrPl, fbErrPl, faErrSePl but also magnetometer for
  // fdErrSePl
  for (i = X; i <= Z; i++) {
    ThErrPl[i] = K12x6[i][0] * gErrSeMi[X] + K12x6[i][1] * gErrSeMi[Y] +
                 K12x6[i][2] * gErrSeMi[Z];
    bErrPl[i] = K12x6[i + 3][0] * gErrSeMi[X] + K12x6[i + 3][1] * gErrSeMi[Y] +
                K12x6[i + 3][2] * gErrSeMi[Z];
    aErrSePl[i] = K12x6[i + 6][0] * gErrSeMi[X] +
                  K12x6[i + 6][1] * gErrSeMi[Y] + K12x6[i + 6][2] * gErrSeMi[Z];
    dErrSePl[i] =
        K12x6[i + 9][0] * gErrSeMi[X] + K12x6[i + 9][1] * gErrSeMi[Y] +
        K12x6[i + 9][2] * gErrSeMi[Z] + K12x6[i + 9][3] * mErrSeMi[X] +
        K12x6[i + 9][4] * mErrSeMi[Y] + K12x6[i + 9][5] * mErrSeMi[Z];
  }

  // set the magnetic jamming flag if there is a significant magnetic error
  // power after calibration
  ftmp = dErrSePl[X] * dErrSePl[X] + dErrSePl[Y] * dErrSePl[Y] +
         dErrSePl[Z] * dErrSePl[Z];
  // iMagJamming = (ValidMagCal) && (ftmp > MagCal->FourBsq);
  iMagJamming = (ValidMagCal) &&
                (ftmp > (DEFAULTB * DEFAULTB * 4.0f)); // TODO: FourBsq....

  // add the remaining magnetic error terms if there is calibration and no
  // magnetic jamming
  if (ValidMagCal && !iMagJamming) {
    for (i = X; i <= Z; i++) {
      ThErrPl[i] += K12x6[i][3] * mErrSeMi[X] + K12x6[i][4] * mErrSeMi[Y] +
                    K12x6[i][5] * mErrSeMi[Z];
      bErrPl[i] += K12x6[i + 3][3] * mErrSeMi[X] +
                   K12x6[i + 3][4] * mErrSeMi[Y] +
                   K12x6[i + 3][5] * mErrSeMi[Z];
      aErrSePl[i] += K12x6[i + 6][3] * mErrSeMi[X] +
                     K12x6[i + 6][4] * mErrSeMi[Y] +
                     K12x6[i + 6][5] * mErrSeMi[Z];
    }
  }

  // *********************************************************************************
  // apply the a posteriori error corrections to the a posteriori state vector
  // *********************************************************************************

  // get the a posteriori delta quaternion
  fQuaternionFromRotationVectorDeg(&Deltaq, ThErrPl, -1.0F);

  // compute the a posteriori orientation quaternion fqPl = fqMi *
  // Deltaq(-thetae+) the resulting quaternion may have negative scalar
  // component q0
  qAeqBxC(&qPl, &qMi, &Deltaq);

  // normalize the a posteriori orientation quaternion to stop error propagation
  // the renormalization function ensures that the scalar component q0 is
  // non-negative
  fqAeqNormqA(&qPl);

  // compute the a posteriori rotation matrix from the a posteriori quaternion
  fRotationMatrixFromQuaternion(RPl, &qPl);

  // compute the rotation vector from the a posteriori quaternion
  fRotationVectorDegFromQuaternion(&qPl, RVecPl);

  // update the a posteriori gyro offset vector b+ and
  // assign the entire linear acceleration error vector to update the linear
  // acceleration
  for (i = X; i <= Z; i++) {
    // b+[k] = b-[k] - be+[k] = b+[k] - be+[k] (deg/s)
    bPl[i] -= bErrPl[i];
    // a+ = a- - ae+ (g, sensor frame)
    aSePl[i] = aSeMi[i] - aErrSePl[i];
  }

  // compute the linear acceleration in the global frame from the accelerometer
  // measurement (sensor frame). de-rotate the accelerometer measurement from
  // the sensor to global frame using the inverse (transpose) of the a
  // posteriori rotation matrix
  aGlPl[X] = RPl[X][X] * Accel[X] + RPl[Y][X] * Accel[Y] + RPl[Z][X] * Accel[Z];
  aGlPl[Y] = RPl[X][Y] * Accel[X] + RPl[Y][Y] * Accel[Y] + RPl[Z][Y] * Accel[Z];
  aGlPl[Z] = RPl[X][Z] * Accel[X] + RPl[Y][Z] * Accel[Y] + RPl[Z][Z] * Accel[Z];
  // remove gravity and correct the sign if the coordinate system is gravity
  // positive / acceleration negative gravity positive NED
  aGlPl[X] = -aGlPl[X];
  aGlPl[Y] = -aGlPl[Y];
  aGlPl[Z] = -(aGlPl[Z] - 1.0F);

  // update the reference geomagnetic vector using magnetic disturbance error if
  // valid calibration and no jamming
  if (ValidMagCal && !iMagJamming) {
    // de-rotate the NED magnetic disturbance error de+ from the sensor to the
    // global reference frame using the inverse (transpose) of the a posteriori
    // rotation matrix
    dErrGlPl[X] = RPl[X][X] * dErrSePl[X] + RPl[Y][X] * dErrSePl[Y] +
                  RPl[Z][X] * dErrSePl[Z];
    dErrGlPl[Z] = RPl[X][Z] * dErrSePl[X] + RPl[Y][Z] * dErrSePl[Y] +
                  RPl[Z][Z] * dErrSePl[Z];

    // compute components of the new geomagnetic vector
    // the north pointing component fadj must always be non-negative
    fopp = mGl[Z] - dErrGlPl[Z];
    fadj = mGl[X] - dErrGlPl[X];
    if (fadj < 0.0F) {
      fadj = 0.0F;
    }
    fhyp = sqrtf(fopp * fopp + fadj * fadj);

    // check for the pathological condition of zero geomagnetic field
    if (fhyp != 0.0F) {
      // compute the sine and cosine of the new geomagnetic vector
      ftmp = 1.0F / fhyp;
      fsindelta = fopp * ftmp;
      fcosdelta = fadj * ftmp;

      // limit the inclination angle between limits to prevent runaway
      if (fsindelta > SINDELTAMAX) {
        fsindelta = SINDELTAMAX;
        fcosdelta = COSDELTAMAX;
      } else if (fsindelta < -SINDELTAMAX) {
        fsindelta = -SINDELTAMAX;
        fcosdelta = COSDELTAMAX;
      }

      // compute the new geomagnetic vector (always north pointing)
      DeltaPl = fasin_deg(fsindelta);
      // mGl[X] = MagCal->B * fcosdelta;  // TODO: MagCal->B
      // mGl[Z] = MagCal->B * fsindelta;
      mGl[X] = DEFAULTB * fcosdelta; // TODO: MagCal->B
      mGl[Z] = DEFAULTB * fsindelta;
    } // end hyp == 0.0F
  }   // end ValidMagCal

  // *********************************************************************************
  // compute the a posteriori Euler angles from the orientation matrix
  // *********************************************************************************

  // calculate the NED Euler angles
  fNEDAnglesDegFromRotationMatrix(RPl, &PhiPl, &ThePl, &PsiPl, &RhoPl, &ChiPl);

  // ***********************************************************************************
  // calculate (symmetric) a posteriori error covariance matrix P+
  // P+ = (I12 - K * C) * P- = (I12 - K * C) * Qw = Qw - K * (C * Qw)
  // both Qw and P+ are used as working arrays in this section
  // at the end of this section, P+ is valid but Qw is over-written
  // ***********************************************************************************

  // set P+ (6x12 scratch sub-matrix) to the product C (6x12) * Qw (12x12)
  // where both C and Qw are sparse and C has a significant number of +1 and -1
  // entries the resulting matrix is sparse but not symmetric
  for (i = 0; i < 6; i++) {
    // initialize pfPPlus12x12ij for current i, j=0
    pfPPlus12x12ij = PPlus12x12[i];

    for (j = 0; j < 12; j++) {
      // zero P+[i][j]
      *pfPPlus12x12ij = 0.0F;

      // initialize pfC6x12ik for current i, k=0
      pfC6x12ik = C6x12[i];

      // initialize pfQw12x12kj for current j, k=0
      pfQw12x12kj = &Qw12x12[0][j];

      // sum matrix products over inner loop over k
      for (k = 0; k < 12; k++) {
        if ((*pfC6x12ik != 0.0F) && (*pfQw12x12kj != 0.0F)) {
          if (*pfC6x12ik == 1.0F)
            *pfPPlus12x12ij += *pfQw12x12kj;
          else if (*pfC6x12ik == -1.0F)
            *pfPPlus12x12ij -= *pfQw12x12kj;
          else
            *pfPPlus12x12ij += *pfC6x12ik * *pfQw12x12kj;
        }

        // update pfC6x12ik and pfQw12x12kj for next iteration of k
        pfC6x12ik++;
        pfQw12x12kj += 12;
      }

      // increment pfPPlus12x12ij for next iteration of j
      pfPPlus12x12ij++;
    }
  }

  // compute P+ = (I12 - K * C) * Qw = Qw - K * (C * Qw) = Qw - K * P+ (6x12
  // sub-matrix) storing result P+ in Qw and over-writing Qw which is OK since
  // Qw is later computed from P+ where working array P+ (6x12 sub-matrix) is
  // sparse but K is not sparse only on and above diagonal terms of P+ are
  // computed since P+ is symmetric
  for (i = 0; i < 12; i++) {
    // initialize pfQw12x12ij for current i, j=i
    pfQw12x12ij = Qw12x12[i] + i;

    for (j = i; j < 12; j++) {
      // initialize pfK12x6ik for current i, k=0
      pfK12x6ik = K12x6[i];

      // initialize pfPPlus12x12kj for current j, k=0
      pfPPlus12x12kj = *PPlus12x12 + j;

      // compute on and above diagonal matrix entry
      for (k = 0; k < 6; k++) {
        // check for non-zero values since P+ (6x12 scratch sub-matrix) is
        // sparse
        if (*pfPPlus12x12kj != 0.0F) {
          *pfQw12x12ij -= *pfK12x6ik * *pfPPlus12x12kj;
        }
        // increment pfK12x6ik and pfPPlus12x12kj for next iteration of k
        pfK12x6ik++;
        pfPPlus12x12kj += 12;
      }

      // increment pfQw12x12ij for next iteration of j
      pfQw12x12ij++;
    }
  }

  // Qw now holds the on and above diagonal elements of P+
  // so perform a simple copy to the all elements of P+
  // after execution of this code P+ is valid but Qw remains invalid
  for (i = 0; i < 12; i++) {
    // initialize pfPPlus12x12ij and pfQw12x12ij for current i, j=i
    pfPPlus12x12ij = PPlus12x12[i] + i;
    pfQw12x12ij = Qw12x12[i] + i;

    // copy the on-diagonal elements and increment pointers to enter loop at
    // j=i+1
    *(pfPPlus12x12ij++) = *(pfQw12x12ij++);

    // loop over above diagonal columns j copying to below-diagonal elements
    for (j = i + 1; j < 12; j++) {
      *(pfPPlus12x12ij++) = PPlus12x12[j][i] = *(pfQw12x12ij++);
    }
  }

  // *********************************************************************************
  // re-create the noise covariance matrix Qw=fn(P+) for the next iteration
  // using the elements of P+ which are now valid
  // Qw was over-written earlier but is here recomputed (all elements)
  // *********************************************************************************

  // zero the matrix Qw
  for (i = 0; i < 12; i++) {
    for (j = 0; j < 12; j++) {
      Qw12x12[i][j] = 0.0F;
    }
  }

  // update the covariance matrix components
  for (i = 0; i < 3; i++) {
    // Qw[th-th-] = Qw[0-2][0-2] = E[th-(th-)^T] = Q[th+th+] + deltat^2 *
    // (Q[b+b+] + (Qwb + QvG) * I)
    Qw12x12[i][i] =
        PPlus12x12[i][i] + deltatsq * (PPlus12x12[i + 3][i + 3] + QwbplusQvG);

    // Qw[b-b-] = Qw[3-5][3-5] = E[b-(b-)^T] = Q[b+b+] + Qwb * I
    Qw12x12[i + 3][i + 3] = PPlus12x12[i + 3][i + 3] + FQWB_9DOF_GBY_KALMAN;

    // Qw[th-b-] = Qw[0-2][3-5] = E[th-(b-)^T] = -deltat * (Q[b+b+] + Qwb * I) =
    // -deltat * Qw[b-b-]
    Qw12x12[i][i + 3] = Qw12x12[i + 3][i] = -deltat * Qw12x12[i + 3][i + 3];

    // Qw[a-a-] = Qw[6-8][6-8] = E[a-(a-)^T] = ca^2 * Q[a+a+] + Qwa * I
    Qw12x12[i + 6][i + 6] =
        casq * PPlus12x12[i + 6][i + 6] + FQWA_9DOF_GBY_KALMAN;

    // Qw[d-d-] = Qw[9-11][9-11] = E[d-(d-)^T] = cd^2 * Q[d+d+] + Qwd * I
    Qw12x12[i + 9][i + 9] =
        cdsq * PPlus12x12[i + 9][i + 9] + FQWD_9DOF_GBY_KALMAN;
  }
}

// compile time constants that are private to this file
#define SMALLQ0                                                                \
  0.01F // limit of quaternion scalar component requiring special algorithm
#define CORRUPTQUAT                                                            \
  0.001F // threshold for deciding rotation quaternion is corrupt
#define SMALLMODULUS 0.01F // limit where rounding errors may appear

// Aerospace NED accelerometer 3DOF tilt function computing rotation matrix fR
void f3DOFTiltNED(float fR[][3], float fGp[]) {
  // the NED self-consistency twist occurs at 90 deg pitch

  // local variables
  int16_t i;           // counter
  float fmodGxyz;      // modulus of the x, y, z accelerometer readings
  float fmodGyz;       // modulus of the y, z accelerometer readings
  float frecipmodGxyz; // reciprocal of modulus
  float ftmp;          // scratch variable

  // compute the accelerometer squared magnitudes
  fmodGyz = fGp[Y] * fGp[Y] + fGp[Z] * fGp[Z];
  fmodGxyz = fmodGyz + fGp[X] * fGp[X];

  // check for freefall special case where no solution is possible
  if (fmodGxyz == 0.0F) {
    f3x3matrixAeqI(fR);
    return;
  }

  // check for vertical up or down gimbal lock case
  if (fmodGyz == 0.0F) {
    f3x3matrixAeqScalar(fR, 0.0F);
    fR[Y][Y] = 1.0F;
    if (fGp[X] >= 0.0F) {
      fR[X][Z] = 1.0F;
      fR[Z][X] = -1.0F;
    } else {
      fR[X][Z] = -1.0F;
      fR[Z][X] = 1.0F;
    }
    return;
  }

  // compute moduli for the general case
  fmodGyz = sqrtf(fmodGyz);
  fmodGxyz = sqrtf(fmodGxyz);
  frecipmodGxyz = 1.0F / fmodGxyz;
  ftmp = fmodGxyz / fmodGyz;

  // normalize the accelerometer reading into the z column
  for (i = X; i <= Z; i++) {
    fR[i][Z] = fGp[i] * frecipmodGxyz;
  }

  // construct x column of orientation matrix
  fR[X][X] = fmodGyz * frecipmodGxyz;
  fR[Y][X] = -fR[X][Z] * fR[Y][Z] * ftmp;
  fR[Z][X] = -fR[X][Z] * fR[Z][Z] * ftmp;

  // // construct y column of orientation matrix
  fR[X][Y] = 0.0F;
  fR[Y][Y] = fR[Z][Z] * ftmp;
  fR[Z][Y] = -fR[Y][Z] * ftmp;
}

// Aerospace NED magnetometer 3DOF flat eCompass function computing rotation
// matrix fR
void f3DOFMagnetometerMatrixNED(float fR[][3], float fBc[]) {
  // local variables
  float fmodBxy; // modulus of the x, y magnetometer readings

  // compute the magnitude of the horizontal (x and y) magnetometer reading
  fmodBxy = sqrtf(fBc[X] * fBc[X] + fBc[Y] * fBc[Y]);

  // check for zero field special case where no solution is possible
  if (fmodBxy == 0.0F) {
    f3x3matrixAeqI(fR);
    return;
  }

  // define the fixed entries in the z row and column
  fR[Z][X] = fR[Z][Y] = fR[X][Z] = fR[Y][Z] = 0.0F;
  fR[Z][Z] = 1.0F;

  // define the remaining entries
  fR[X][X] = fR[Y][Y] = fBc[X] / fmodBxy;
  fR[Y][X] = fBc[Y] / fmodBxy;
  fR[X][Y] = -fR[Y][X];
}

// NED: 6DOF e-Compass function computing rotation matrix fR
static void feCompassNED(float fR[][3], float *pfDelta, const float fBc[],
                         const float fGp[]) {
  // local variables
  float fmod[3]; // column moduli
  float fmodBc;  // modulus of Bc
  float fGdotBc; // dot product of vectors G.Bc
  float ftmp;    // scratch variable
  int8_t i, j;   // loop counters

  // set the inclination angle to zero in case it is not computed later
  *pfDelta = 0.0F;

  // place the un-normalized gravity and geomagnetic vectors into the rotation
  // matrix z and x axes
  for (i = X; i <= Z; i++) {
    fR[i][Z] = fGp[i];
    fR[i][X] = fBc[i];
  }

  // set y vector to vector product of z and x vectors
  fR[X][Y] = fR[Y][Z] * fR[Z][X] - fR[Z][Z] * fR[Y][X];
  fR[Y][Y] = fR[Z][Z] * fR[X][X] - fR[X][Z] * fR[Z][X];
  fR[Z][Y] = fR[X][Z] * fR[Y][X] - fR[Y][Z] * fR[X][X];

  // set x vector to vector product of y and z vectors
  fR[X][X] = fR[Y][Y] * fR[Z][Z] - fR[Z][Y] * fR[Y][Z];
  fR[Y][X] = fR[Z][Y] * fR[X][Z] - fR[X][Y] * fR[Z][Z];
  fR[Z][X] = fR[X][Y] * fR[Y][Z] - fR[Y][Y] * fR[X][Z];

  // calculate the rotation matrix column moduli
  fmod[X] =
      sqrtf(fR[X][X] * fR[X][X] + fR[Y][X] * fR[Y][X] + fR[Z][X] * fR[Z][X]);
  fmod[Y] =
      sqrtf(fR[X][Y] * fR[X][Y] + fR[Y][Y] * fR[Y][Y] + fR[Z][Y] * fR[Z][Y]);
  fmod[Z] =
      sqrtf(fR[X][Z] * fR[X][Z] + fR[Y][Z] * fR[Y][Z] + fR[Z][Z] * fR[Z][Z]);

  // normalize the rotation matrix columns
  if (!((fmod[X] == 0.0F) || (fmod[Y] == 0.0F) || (fmod[Z] == 0.0F))) {
    // loop over columns j
    for (j = X; j <= Z; j++) {
      ftmp = 1.0F / fmod[j];
      // loop over rows i
      for (i = X; i <= Z; i++) {
        // normalize by the column modulus
        fR[i][j] *= ftmp;
      }
    }
  } else {
    // no solution is possible to set rotation to identity matrix
    f3x3matrixAeqI(fR);
    return;
  }

  // compute the geomagnetic inclination angle
  fmodBc = sqrtf(fBc[X] * fBc[X] + fBc[Y] * fBc[Y] + fBc[Z] * fBc[Z]);
  fGdotBc = fGp[X] * fBc[X] + fGp[Y] * fBc[Y] + fGp[Z] * fBc[Z];
  if (!((fmod[Z] == 0.0F) || (fmodBc == 0.0F))) {
    *pfDelta = fasin_deg(fGdotBc / (fmod[Z] * fmodBc));
  }
}

// extract the NED angles in degrees from the NED rotation matrix
static void fNEDAnglesDegFromRotationMatrix(float R[][3], float *pfPhiDeg,
                                            float *pfTheDeg, float *pfPsiDeg,
                                            float *pfRhoDeg, float *pfChiDeg) {
  // calculate the pitch angle -90.0 <= Theta <= 90.0 deg
  *pfTheDeg = fasin_deg(-R[X][Z]);

  // calculate the roll angle range -180.0 <= Phi < 180.0 deg
  *pfPhiDeg = fatan2_deg(R[Y][Z], R[Z][Z]);

  // map +180 roll onto the functionally equivalent -180 deg roll
  if (*pfPhiDeg == 180.0F) {
    *pfPhiDeg = -180.0F;
  }

  // calculate the yaw (compass) angle 0.0 <= Psi < 360.0 deg
  if (*pfTheDeg == 90.0F) {
    // vertical upwards gimbal lock case
    *pfPsiDeg = fatan2_deg(R[Z][Y], R[Y][Y]) + *pfPhiDeg;
  } else if (*pfTheDeg == -90.0F) {
    // vertical downwards gimbal lock case
    *pfPsiDeg = fatan2_deg(-R[Z][Y], R[Y][Y]) - *pfPhiDeg;
  } else {
    // general case
    *pfPsiDeg = fatan2_deg(R[X][Y], R[X][X]);
  }

  // map yaw angle Psi onto range 0.0 <= Psi < 360.0 deg
  if (*pfPsiDeg < 0.0F) {
    *pfPsiDeg += 360.0F;
  }

  // check for rounding errors mapping small negative angle to 360 deg
  if (*pfPsiDeg >= 360.0F) {
    *pfPsiDeg = 0.0F;
  }

  // for NED, the compass heading Rho equals the yaw angle Psi
  *pfRhoDeg = *pfPsiDeg;

  // calculate the tilt angle from vertical Chi (0 <= Chi <= 180 deg)
  *pfChiDeg = facos_deg(R[Z][Z]);

  return;
}

// computes normalized rotation quaternion from a rotation vector (deg)
static void fQuaternionFromRotationVectorDeg(Quaternion_t *pq,
                                             const float rvecdeg[],
                                             float fscaling) {
  float fetadeg;    // rotation angle (deg)
  float fetarad;    // rotation angle (rad)
  float fetarad2;   // eta (rad)^2
  float fetarad4;   // eta (rad)^4
  float sinhalfeta; // sin(eta/2)
  float fvecsq;     // q1^2+q2^2+q3^2
  float ftmp;       // scratch variable

  // compute the scaled rotation angle eta (deg) which can be both positve or
  // negative
  fetadeg = fscaling * sqrtf(rvecdeg[X] * rvecdeg[X] + rvecdeg[Y] * rvecdeg[Y] +
                             rvecdeg[Z] * rvecdeg[Z]);
  fetarad = fetadeg * FDEGTORAD;
  fetarad2 = fetarad * fetarad;

  // calculate the sine and cosine using small angle approximations or exact
  // angles under sqrt(0.02)=0.141 rad is 8.1 deg and 1620 deg/s (=936deg/s in 3
  // axes) at 200Hz and 405 deg/s at 50Hz
  if (fetarad2 <= 0.02F) {
    // use MacLaurin series up to and including third order
    sinhalfeta = fetarad * (0.5F - ONEOVER48 * fetarad2);
  } else if (fetarad2 <= 0.06F) {
    // use MacLaurin series up to and including fifth order
    // angles under sqrt(0.06)=0.245 rad is 14.0 deg and 2807 deg/s (=1623deg/s
    // in 3 axes) at 200Hz and 703 deg/s at 50Hz
    fetarad4 = fetarad2 * fetarad2;
    sinhalfeta =
        fetarad * (0.5F - ONEOVER48 * fetarad2 + ONEOVER3840 * fetarad4);
  } else {
    // use exact calculation
    sinhalfeta = (float)sinf(0.5F * fetarad);
  }

  // compute the vector quaternion components q1, q2, q3
  if (fetadeg != 0.0F) {
    // general case with non-zero rotation angle
    ftmp = fscaling * sinhalfeta / fetadeg;
    pq->q1 = rvecdeg[X] * ftmp; // q1 = nx * sin(eta/2)
    pq->q2 = rvecdeg[Y] * ftmp; // q2 = ny * sin(eta/2)
    pq->q3 = rvecdeg[Z] * ftmp; // q3 = nz * sin(eta/2)
  } else {
    // zero rotation angle giving zero vector component
    pq->q1 = pq->q2 = pq->q3 = 0.0F;
  }

  // compute the scalar quaternion component q0 by explicit normalization
  // taking care to avoid rounding errors giving negative operand to sqrt
  fvecsq = pq->q1 * pq->q1 + pq->q2 * pq->q2 + pq->q3 * pq->q3;
  if (fvecsq <= 1.0F) {
    // normal case
    pq->q0 = sqrtf(1.0F - fvecsq);
  } else {
    // rounding errors are present
    pq->q0 = 0.0F;
  }
}

// compute the orientation quaternion from a 3x3 rotation matrix
static void fQuaternionFromRotationMatrix(float R[][3], Quaternion_t *pq) {
  float fq0sq;    // q0^2
  float recip4q0; // 1/4q0

  // the quaternion is not explicitly normalized in this function on the
  // assumption that it is supplied with a normalized rotation matrix. if the
  // rotation matrix is normalized then the quaternion will also be normalized
  // even if the case of small q0

  // get q0^2 and q0
  fq0sq = 0.25F * (1.0F + R[X][X] + R[Y][Y] + R[Z][Z]);
  pq->q0 = sqrtf(fabs(fq0sq));

  // normal case when q0 is not small meaning rotation angle not near 180 deg
  if (pq->q0 > SMALLQ0) {
    // calculate q1 to q3 (general case)
    recip4q0 = 0.25F / pq->q0;
    pq->q1 = recip4q0 * (R[Y][Z] - R[Z][Y]);
    pq->q2 = recip4q0 * (R[Z][X] - R[X][Z]);
    pq->q3 = recip4q0 * (R[X][Y] - R[Y][X]);
  } else {
    // special case of near 180 deg corresponds to nearly symmetric matrix
    // which is not numerically well conditioned for division by small q0
    // instead get absolute values of q1 to q3 from leading diagonal
    pq->q1 = sqrtf(fabs(0.5F * (1.0F + R[X][X]) - fq0sq));
    pq->q2 = sqrtf(fabs(0.5F * (1.0F + R[Y][Y]) - fq0sq));
    pq->q3 = sqrtf(fabs(0.5F * (1.0F + R[Z][Z]) - fq0sq));

    // correct the signs of q1 to q3 by examining the signs of differenced
    // off-diagonal terms
    if ((R[Y][Z] - R[Z][Y]) < 0.0F)
      pq->q1 = -pq->q1;
    if ((R[Z][X] - R[X][Z]) < 0.0F)
      pq->q2 = -pq->q2;
    if ((R[X][Y] - R[Y][X]) < 0.0F)
      pq->q3 = -pq->q3;
  }
}

// compute the rotation matrix from an orientation quaternion
static void fRotationMatrixFromQuaternion(float R[][3],
                                          const Quaternion_t *pq) {
  float f2q;
  float f2q0q0, f2q0q1, f2q0q2, f2q0q3;
  float f2q1q1, f2q1q2, f2q1q3;
  float f2q2q2, f2q2q3;
  float f2q3q3;

  // calculate products
  f2q = 2.0F * pq->q0;
  f2q0q0 = f2q * pq->q0;
  f2q0q1 = f2q * pq->q1;
  f2q0q2 = f2q * pq->q2;
  f2q0q3 = f2q * pq->q3;
  f2q = 2.0F * pq->q1;
  f2q1q1 = f2q * pq->q1;
  f2q1q2 = f2q * pq->q2;
  f2q1q3 = f2q * pq->q3;
  f2q = 2.0F * pq->q2;
  f2q2q2 = f2q * pq->q2;
  f2q2q3 = f2q * pq->q3;
  f2q3q3 = 2.0F * pq->q3 * pq->q3;

  // calculate the rotation matrix assuming the quaternion is normalized
  R[X][X] = f2q0q0 + f2q1q1 - 1.0F;
  R[X][Y] = f2q1q2 + f2q0q3;
  R[X][Z] = f2q1q3 - f2q0q2;
  R[Y][X] = f2q1q2 - f2q0q3;
  R[Y][Y] = f2q0q0 + f2q2q2 - 1.0F;
  R[Y][Z] = f2q2q3 + f2q0q1;
  R[Z][X] = f2q1q3 + f2q0q2;
  R[Z][Y] = f2q2q3 - f2q0q1;
  R[Z][Z] = f2q0q0 + f2q3q3 - 1.0F;
}

// function calculate the rotation vector from a rotation matrix
void fRotationVectorDegFromRotationMatrix(float R[][3], float rvecdeg[]) {
  float ftrace;   // trace of the rotation matrix
  float fetadeg;  // rotation angle eta (deg)
  float fmodulus; // modulus of axis * angle vector = 2|sin(eta)|
  float ftmp;     // scratch variable

  // calculate the trace of the rotation matrix = 1+2cos(eta) in range -1 to +3
  // and eta (deg) in range 0 to 180 deg inclusive
  // checking for rounding errors that might take the trace outside this range
  ftrace = R[X][X] + R[Y][Y] + R[Z][Z];
  if (ftrace >= 3.0F) {
    fetadeg = 0.0F;
  } else if (ftrace <= -1.0F) {
    fetadeg = 180.0F;
  } else {
    fetadeg = acosf(0.5F * (ftrace - 1.0F)) * FRADTODEG;
  }

  // set the rvecdeg vector to differences across the diagonal = 2*n*sin(eta)
  // and calculate its modulus equal to 2|sin(eta)|
  // the modulus approaches zero near 0 and 180 deg (when sin(eta) approaches
  // zero)
  rvecdeg[X] = R[Y][Z] - R[Z][Y];
  rvecdeg[Y] = R[Z][X] - R[X][Z];
  rvecdeg[Z] = R[X][Y] - R[Y][X];
  fmodulus = sqrtf(rvecdeg[X] * rvecdeg[X] + rvecdeg[Y] * rvecdeg[Y] +
                   rvecdeg[Z] * rvecdeg[Z]);

  // normalize the rotation vector for general, 0 deg and 180 deg rotation cases
  if (fmodulus > SMALLMODULUS) {
    // general case away from 0 and 180 deg rotation
    ftmp = fetadeg / fmodulus;
    rvecdeg[X] *= ftmp; // set x component to eta(deg) * nx
    rvecdeg[Y] *= ftmp; // set y component to eta(deg) * ny
    rvecdeg[Z] *= ftmp; // set z component to eta(deg) * nz
  } else if (ftrace >= 0.0F) {
    // near 0 deg rotation (trace = 3): matrix is nearly identity matrix
    // R[Y][Z]-R[Z][Y]=2*nx*eta(rad) and similarly for other components
    ftmp = 0.5F * FRADTODEG;
    rvecdeg[X] *= ftmp;
    rvecdeg[Y] *= ftmp;
    rvecdeg[Z] *= ftmp;
  } else {
    // near 180 deg (trace = -1): matrix is nearly symmetric
    // calculate the absolute value of the components of the axis-angle vector
    rvecdeg[X] = 180.0F * sqrtf(fabs(0.5F * (R[X][X] + 1.0F)));
    rvecdeg[Y] = 180.0F * sqrtf(fabs(0.5F * (R[Y][Y] + 1.0F)));
    rvecdeg[Z] = 180.0F * sqrtf(fabs(0.5F * (R[Z][Z] + 1.0F)));

    // correct the signs of the three components by examining the signs of
    // differenced off-diagonal terms
    if ((R[Y][Z] - R[Z][Y]) < 0.0F)
      rvecdeg[X] = -rvecdeg[X];
    if ((R[Z][X] - R[X][Z]) < 0.0F)
      rvecdeg[Y] = -rvecdeg[Y];
    if ((R[X][Y] - R[Y][X]) < 0.0F)
      rvecdeg[Z] = -rvecdeg[Z];
  }
}

// computes rotation vector (deg) from rotation quaternion
static void fRotationVectorDegFromQuaternion(Quaternion_t *pq,
                                             float rvecdeg[]) {
  float fetarad;    // rotation angle (rad)
  float fetadeg;    // rotation angle (deg)
  float sinhalfeta; // sin(eta/2)
  float ftmp;       // scratch variable

  // calculate the rotation angle in the range 0 <= eta < 360 deg
  if ((pq->q0 >= 1.0F) || (pq->q0 <= -1.0F)) {
    // rotation angle is 0 deg or 2*180 deg = 360 deg = 0 deg
    fetarad = 0.0F;
    fetadeg = 0.0F;
  } else {
    // general case returning 0 < eta < 360 deg
    fetarad = 2.0F * acosf(pq->q0);
    fetadeg = fetarad * FRADTODEG;
  }

  // map the rotation angle onto the range -180 deg <= eta < 180 deg
  if (fetadeg >= 180.0F) {
    fetadeg -= 360.0F;
    fetarad = fetadeg * FDEGTORAD;
  }

  // calculate sin(eta/2) which will be in the range -1 to +1
  sinhalfeta = (float)sinf(0.5F * fetarad);

  // calculate the rotation vector (deg)
  if (sinhalfeta == 0.0F) {
    // the rotation angle eta is zero and the axis is irrelevant
    rvecdeg[X] = rvecdeg[Y] = rvecdeg[Z] = 0.0F;
  } else {
    // general case with non-zero rotation angle
    ftmp = fetadeg / sinhalfeta;
    rvecdeg[X] = pq->q1 * ftmp;
    rvecdeg[Y] = pq->q2 * ftmp;
    rvecdeg[Z] = pq->q3 * ftmp;
  }
}

#if 0
// function low pass filters an orientation quaternion and computes virtual gyro rotation rate
void fLPFOrientationQuaternion(Quaternion_t *pq, Quaternion_t *pLPq, float flpf, float fdeltat,
    float fOmega[], int32_t loopcounter)
{
  // local variables
  Quaternion_t fdeltaq;     // delta rotation quaternion
  float rvecdeg[3];       // rotation vector (deg)
  float ftmp;           // scratch variable

  // initialize delay line on first pass: LPq[n]=q[n]
  if (loopcounter == 0) {
    *pLPq = *pq;
  }

  // set fdeltaqn to the delta rotation quaternion conjg(fLPq[n-1) . fqn
  fdeltaq = qconjgAxB(pLPq, pq);
  if (fdeltaq.q0 < 0.0F) {
    fdeltaq.q0 = -fdeltaq.q0;
    fdeltaq.q1 = -fdeltaq.q1;
    fdeltaq.q2 = -fdeltaq.q2;
    fdeltaq.q3 = -fdeltaq.q3;
  }

  // set ftmp to a scaled lpf value which equals flpf in the limit of small rotations (q0=1)
  // but which rises as the delta rotation angle increases (q0 tends to zero)
  ftmp = flpf + 0.75F * (1.0F - fdeltaq.q0);
  if (ftmp > 1.0F) {
    ftmp = 1.0F;
  }

  // scale the delta rotation by the corrected lpf value
  fdeltaq.q1 *= ftmp;
  fdeltaq.q2 *= ftmp;
  fdeltaq.q3 *= ftmp;

  // compute the scalar component q0
  ftmp = fdeltaq.q1 * fdeltaq.q1 + fdeltaq.q2 * fdeltaq.q2 + fdeltaq.q3 * fdeltaq.q3;
  if (ftmp <= 1.0F) {
    // normal case
    fdeltaq.q0 = sqrtf(1.0F - ftmp);
  } else {
    // rounding errors present so simply set scalar component to 0
    fdeltaq.q0 = 0.0F;
  }

  // calculate the delta rotation vector from fdeltaqn and the virtual gyro angular velocity (deg/s)
  fRotationVectorDegFromQuaternion(&fdeltaq, rvecdeg);
  ftmp = 1.0F / fdeltat;
  fOmega[X] = rvecdeg[X] * ftmp;
  fOmega[Y] = rvecdeg[Y] * ftmp;
  fOmega[Z] = rvecdeg[Z] * ftmp;

  // set LPq[n] = LPq[n-1] . deltaq[n]
  qAeqAxB(pLPq, &fdeltaq);

  // renormalize the low pass filtered quaternion to prevent error accumulation
  // the renormalization function ensures that q0 is non-negative
  fqAeqNormqA(pLPq);
}

// function low pass filters a scalar
void fLPFScalar(float *pfS, float *pfLPS, float flpf, int32_t loopcounter)
{
  // set S[LP,n]=S[n] on first pass
  if (loopcounter == 0) {
    *pfLPS = *pfS;
  }

  // apply the exponential low pass filter
  *pfLPS += flpf * (*pfS - *pfLPS);
}
#endif

// function compute the quaternion product qA * qB
static void qAeqBxC(Quaternion_t *pqA, const Quaternion_t *pqB,
                    const Quaternion_t *pqC) {
  pqA->q0 = pqB->q0 * pqC->q0 - pqB->q1 * pqC->q1 - pqB->q2 * pqC->q2 -
            pqB->q3 * pqC->q3;
  pqA->q1 = pqB->q0 * pqC->q1 + pqB->q1 * pqC->q0 + pqB->q2 * pqC->q3 -
            pqB->q3 * pqC->q2;
  pqA->q2 = pqB->q0 * pqC->q2 - pqB->q1 * pqC->q3 + pqB->q2 * pqC->q0 +
            pqB->q3 * pqC->q1;
  pqA->q3 = pqB->q0 * pqC->q3 + pqB->q1 * pqC->q2 - pqB->q2 * pqC->q1 +
            pqB->q3 * pqC->q0;
}

// function compute the quaternion product qA = qA * qB
static void qAeqAxB(Quaternion_t *pqA, const Quaternion_t *pqB) {
  Quaternion_t qProd;

  // perform the quaternion product
  qProd.q0 = pqA->q0 * pqB->q0 - pqA->q1 * pqB->q1 - pqA->q2 * pqB->q2 -
             pqA->q3 * pqB->q3;
  qProd.q1 = pqA->q0 * pqB->q1 + pqA->q1 * pqB->q0 + pqA->q2 * pqB->q3 -
             pqA->q3 * pqB->q2;
  qProd.q2 = pqA->q0 * pqB->q2 - pqA->q1 * pqB->q3 + pqA->q2 * pqB->q0 +
             pqA->q3 * pqB->q1;
  qProd.q3 = pqA->q0 * pqB->q3 + pqA->q1 * pqB->q2 - pqA->q2 * pqB->q1 +
             pqA->q3 * pqB->q0;

  // copy the result back into qA
  *pqA = qProd;
}

#if 0
// function compute the quaternion product conjg(qA) * qB
static Quaternion_t qconjgAxB(const Quaternion_t *pqA, const Quaternion_t *pqB)
{
  Quaternion_t qProd;

  qProd.q0 = pqA->q0 * pqB->q0 + pqA->q1 * pqB->q1 + pqA->q2 * pqB->q2 + pqA->q3 * pqB->q3;
  qProd.q1 = pqA->q0 * pqB->q1 - pqA->q1 * pqB->q0 - pqA->q2 * pqB->q3 + pqA->q3 * pqB->q2;
  qProd.q2 = pqA->q0 * pqB->q2 + pqA->q1 * pqB->q3 - pqA->q2 * pqB->q0 - pqA->q3 * pqB->q1;
  qProd.q3 = pqA->q0 * pqB->q3 - pqA->q1 * pqB->q2 + pqA->q2 * pqB->q1 - pqA->q3 * pqB->q0;

  return qProd;
}
#endif

// function normalizes a rotation quaternion and ensures q0 is non-negative
static void fqAeqNormqA(Quaternion_t *pqA) {
  float fNorm; // quaternion Norm

  // calculate the quaternion Norm
  fNorm = sqrtf(pqA->q0 * pqA->q0 + pqA->q1 * pqA->q1 + pqA->q2 * pqA->q2 +
                pqA->q3 * pqA->q3);
  if (fNorm > CORRUPTQUAT) {
    // general case
    fNorm = 1.0F / fNorm;
    pqA->q0 *= fNorm;
    pqA->q1 *= fNorm;
    pqA->q2 *= fNorm;
    pqA->q3 *= fNorm;
  } else {
    // return with identity quaternion since the quaternion is corrupted
    pqA->q0 = 1.0F;
    pqA->q1 = pqA->q2 = pqA->q3 = 0.0F;
  }

  // correct a negative scalar component if the function was called with
  // negative q0
  if (pqA->q0 < 0.0F) {
    pqA->q0 = -pqA->q0;
    pqA->q1 = -pqA->q1;
    pqA->q2 = -pqA->q2;
    pqA->q3 = -pqA->q3;
  }
}

// set a quaternion to the unit quaternion
static void fqAeq1(Quaternion_t *pqA) {
  pqA->q0 = 1.0F;
  pqA->q1 = pqA->q2 = pqA->q3 = 0.0F;
}

// function returns an approximation to angle(deg)=asin(x) for x in the range -1
// <= x <= 1 and returns -90 <= angle <= 90 deg maximum error is 10.29E-6 deg
static float fasin_deg(float x) {
  // for robustness, check for invalid argument
  if (x >= 1.0F)
    return 90.0F;
  if (x <= -1.0F)
    return -90.0F;

  // call the atan which will return an angle in the correct range -90 to 90 deg
  // this line cannot fail from division by zero or negative square root since
  // |x| < 1
  return (fatan_deg(x / sqrtf(1.0F - x * x)));
}

// function returns an approximation to angle(deg)=acos(x) for x in the range -1
// <= x <= 1 and returns 0 <= angle <= 180 deg maximum error is 14.67E-6 deg
static float facos_deg(float x) {
  // for robustness, check for invalid arguments
  if (x >= 1.0F)
    return 0.0F;
  if (x <= -1.0F)
    return 180.0F;

  // call the atan which will return an angle in the incorrect range -90 to 90
  // deg these lines cannot fail from division by zero or negative square root
  if (x == 0.0F)
    return 90.0F;
  if (x > 0.0F)
    return fatan_deg((sqrtf(1.0F - x * x) / x));
  return 180.0F + fatan_deg((sqrtf(1.0F - x * x) / x));
}

// function returns angle in range -90 to 90 deg
// maximum error is 9.84E-6 deg
static float fatan_deg(float x) {
  float fangledeg;     // compute computed (deg)
  int8_t ixisnegative; // argument x is negative
  int8_t ixexceeds1;   // argument x is greater than 1.0
  int8_t ixmapped;     // argument in range tan(15 deg) to tan(45 deg)=1.0

#define TAN15DEG 0.26794919243F // tan(15 deg) = 2 - sqrt(3)
#define TAN30DEG 0.57735026919F // tan(30 deg) = 1/sqrt(3)

  // reset all flags
  ixisnegative = ixexceeds1 = ixmapped = 0;

  // test for negative argument to allow use of tan(-x)=-tan(x)
  if (x < 0.0F) {
    x = -x;
    ixisnegative = 1;
  }

  // test for argument above 1 to allow use of atan(x)=pi/2-atan(1/x)
  if (x > 1.0F) {
    x = 1.0F / x;
    ixexceeds1 = 1;
  }

  // at this point, x is in the range 0 to 1 inclusive
  // map argument onto range -tan(15 deg) to tan(15 deg)
  // using tan(angle-30deg) = (tan(angle)-tan(30deg)) / (1 +
  // tan(angle)tan(30deg)) tan(15deg) maps to tan(-15 deg) = -tan(15 deg)
  // 1. maps to (sqrt(3) - 1) / (sqrt(3) + 1) = 2 - sqrt(3) = tan(15 deg)
  if (x > TAN15DEG) {
    x = (x - TAN30DEG) / (1.0F + TAN30DEG * x);
    ixmapped = 1;
  }

  // call the atan estimator to obtain -15 deg <= angle <= 15 deg
  fangledeg = fatan_15deg(x);

  // undo the distortions applied earlier to obtain -90 deg <= angle <= 90 deg
  if (ixmapped)
    fangledeg += 30.0F;
  if (ixexceeds1)
    fangledeg = 90.0F - fangledeg;
  if (ixisnegative)
    fangledeg = -fangledeg;

  return (fangledeg);
}

// function returns approximate atan2 angle in range -180 to 180 deg
// maximum error is 14.58E-6 deg
static float fatan2_deg(float y, float x) {
  // check for zero x to avoid division by zero
  if (x == 0.0F) {
    // return 90 deg for positive y
    if (y > 0.0F)
      return 90.0F;
    // return -90 deg for negative y
    if (y < 0.0F)
      return -90.0F;
    // otherwise y= 0.0 and return 0 deg (invalid arguments)
    return 0.0F;
  }

  // from here onwards, x is guaranteed to be non-zero
  // compute atan2 for quadrant 1 (0 to 90 deg) and quadrant 4 (-90 to 0 deg)
  if (x > 0.0F)
    return (fatan_deg(y / x));
  // compute atan2 for quadrant 2 (90 to 180 deg)
  if ((x < 0.0F) && (y > 0.0F))
    return (180.0F + fatan_deg(y / x));
  // compute atan2 for quadrant 3 (-180 to -90 deg)
  return (-180.0F + fatan_deg(y / x));
}

// approximation to inverse tan function (deg) for x in range
// -tan(15 deg) to tan(15 deg) giving an output -15 deg <= angle <= 15 deg
// using modified Pade[3/2] approximation
static float fatan_15deg(float x) {
  float x2; // x^2

#define PADE_A                                                                 \
  96.644395816F // theoretical Pade[3/2] value is 5/3*180/PI=95.49296
#define PADE_B                                                                 \
  25.086941612F // theoretical Pade[3/2] value is 4/9*180/PI=25.46479
#define PADE_C 1.6867633134F // theoretical Pade[3/2] value is 5/3=1.66667

  // compute the approximation to the inverse tangent
  // the function is anti-symmetric as required for positive and negative
  // arguments
  x2 = x * x;
  return (x * (PADE_A + x2 * PADE_B) / (PADE_C + x2));
}
