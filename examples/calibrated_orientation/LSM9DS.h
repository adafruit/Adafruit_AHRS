#include <Adafruit_LSM9DS1.h>
Adafruit_LSM9DS1 lsm9ds = Adafruit_LSM9DS1();

// Or if you have the older LSM9DS0
//#include <Adafruit_LSM9DS0.h>
// Adafruit_LSM9DS0 lsm9ds = Adafruit_LSM9DS0();

bool init_sensors(void) {
  if (!lsm9ds.begin()) {
    return false;
  }
  accelerometer = &lsm9ds.getAccel();
  gyroscope = &lsm9ds.getGyro();
  magnetometer = &lsm9ds.getMag();

  return true;
}

void setup_sensors(void) {
  // set lowest range
#ifdef __LSM9DS0_H__
  lsm9ds.setupAccel(lsm9ds.LSM9DS0_ACCELRANGE_2G);
  lsm9ds.setupMag(lsm9ds.LSM9DS0_MAGGAIN_4GAUSS);
  lsm9ds.setupGyro(lsm9ds.LSM9DS0_GYROSCALE_245DPS);
#else
  lsm9ds.setupAccel(lsm9ds.LSM9DS1_ACCELRANGE_2G);
  lsm9ds.setupMag(lsm9ds.LSM9DS1_MAGGAIN_4GAUSS);
  lsm9ds.setupGyro(lsm9ds.LSM9DS1_GYROSCALE_245DPS);
#endif
}
