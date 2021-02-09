/*!
 * @file Adafruit_AHRS
 *
 * @mainpage Adafruit AHRS
 *
 * @section intro_sec Introduction
 *
 * This library lets you take an accelerometer, gyroscope and magnetometer
 * and combine the data to create orientation data.
 *
 * Options are Mahony (lowest memory/computation),
 * Madgwick (fair memory/computation), and NXP fusion/Kalman (highest).
 *
 * While in theory these can run on an Arduino UNO/Atmega328P we really
 * recommend a SAMD21 or better. Having single-instruction floating point
 * multiply and plenty of RAM will help a lot!
 */

#ifndef __ADAFRUIT_AHRS_H_
#define __ADAFRUIT_AHRS_H_

#include <Adafruit_AHRS_Madgwick.h>
#include <Adafruit_AHRS_Mahony.h>
#include <Adafruit_AHRS_NXPFusion.h>

#endif // __ADAFRUIT_AHRS_H_
