#include <Adafruit_LIS3MDL.h>
Adafruit_LIS3MDL lis3mdl;

// Can change this to be LSM6DS33 or whatever ya like
// For (older) Feather Sense with LSM6DS33, use this:
#include <Adafruit_LSM6DS33.h>
Adafruit_LSM6DS33 lsm6ds;
// For (newer) Feather Sense with LSM6DS3TR-C, use this:
// #include <Adafruit_LSM6DS3TRC.h>
// Adafruit_LSM6DS3TRC lsm6ds;

bool init_sensors(void) {
  if (!lsm6ds.begin_I2C() || !lis3mdl.begin_I2C()) {
    return false;
  }
  accelerometer = lsm6ds.getAccelerometerSensor();
  gyroscope = lsm6ds.getGyroSensor();
  magnetometer = &lis3mdl;

  return true;
}

void setup_sensors(void) {
  // set lowest range
  lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);

  // set slightly above refresh rate
  lsm6ds.setAccelDataRate(LSM6DS_RATE_104_HZ);
  lsm6ds.setGyroDataRate(LSM6DS_RATE_104_HZ);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_1000_HZ);
  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
}
