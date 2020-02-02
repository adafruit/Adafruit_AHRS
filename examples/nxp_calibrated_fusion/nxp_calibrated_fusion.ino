// Full orientation sensing using NXP's advanced sensor fusion algorithm.
//
// You *must* perform a magnetic calibration before this code will work.
//
// To view this data, use the Arduino Serial Monitor to watch the
// scrolling angles, or run the OrientationVisualiser example in Processing.
// Based on  https://github.com/PaulStoffregen/NXPMotionSense with adjustments
// to Adafruit Unified Sensor interface

#include <Adafruit_LSM6DS33.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>

uint32_t timestamp;

// change to any 
Adafruit_LSM6DS33 lsm6ds33;
Adafruit_LIS3MDL lis3mdl;

Adafruit_Sensor_Calibration cal;

// pick your filter! slower == better quality output

//Adafruit_NXPSensorFusion filter; // slowest
//Adafruit_Madgwick filter;  // faster than NXP
Adafruit_Mahony filter;  // fastest

#define FILTER_UPDATE_RATE_HZ 100

void setup() {
  Serial.begin(115200);
  while (!Serial) yield();

  if (!cal.begin()) {
    Serial.println("Failed to initialize calibration helper");
  } else if (! cal.loadCalibration()) {
    Serial.println("No calibration loaded/found");
  }

  if (!lsm6ds33.begin_I2C() || ! lis3mdl.begin_I2C()) {
    Serial.println("Failed to find LSM6DS33/LIS3MDL chips");
    while (1) {
      delay(10);
    }
  }
  // set lowest range
  lsm6ds33.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  lsm6ds33.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);

  // set slightly above refresh rate
  lsm6ds33.setAccelDataRate(LSM6DS_RATE_104_HZ);
  lsm6ds33.setGyroDataRate(LSM6DS_RATE_104_HZ);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_1000_HZ);
  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);

  filter.begin(FILTER_UPDATE_RATE_HZ);
  timestamp = millis();
}


void loop() {
  float roll, pitch, heading;
  float gx, gy, gz;

  if ((millis() - timestamp) < (1000 / FILTER_UPDATE_RATE_HZ)) {
    return;
  }
  timestamp = millis();
  // Read the motion sensors
  sensors_event_t accel, gyro, temp, mag;
  lsm6ds33.getEvent(&accel, &gyro, &temp);
  lis3mdl.getEvent(&mag);

  cal.calibrate(mag);
  cal.calibrate(accel);
  cal.calibrate(gyro);
  // Gyroscope needs to be converted from Rad/s to Degree/s
  // the rest are not unit-important
  gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
  gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
  gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;
  
  Serial.print("Raw: ");
  Serial.print(accel.acceleration.x, 4); Serial.print(", ");
  Serial.print(accel.acceleration.y, 4); Serial.print(", ");
  Serial.print(accel.acceleration.z, 4); Serial.print(", ");
  Serial.print(gx, 4); Serial.print(", ");
  Serial.print(gy, 4); Serial.print(", ");
  Serial.print(gz, 4); Serial.print(", ");
  Serial.print(mag.magnetic.x, 4); Serial.print(", ");
  Serial.print(mag.magnetic.y, 4); Serial.print(", ");
  Serial.print(mag.magnetic.z, 4); Serial.println("");

  // Update the SensorFusion filter
  filter.update(gx, gy, gz, 
                accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, 
                mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);

  // print the heading, pitch and roll
  roll = filter.getRoll();
  pitch = filter.getPitch();
  heading = filter.getYaw();
  Serial.print("Orientation: ");
  Serial.print(heading);
  Serial.print(" ");
  Serial.print(pitch);
  Serial.print(" ");
  Serial.println(roll);
  Serial.print("Took "); Serial.print(millis()-timestamp); Serial.println(" ms");
}