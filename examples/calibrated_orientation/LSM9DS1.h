#include <Adafruit_LSM9DS1.h>
Adafruit_LSM9DS1 lsm9ds1 = Adafruit_LSM9DS1();

bool init_sensors(void) {
  if (!lsm9ds1.begin()) {
    return false;
  }
  accelerometer = &lsm9ds1.getAccel();
  gyroscope = &lsm9ds1.getGyro();
  magnetometer = &lsm9ds1.getMag();

  return true;
}

void setup_sensors(void) {
  // set lowest range
  lsm9ds1.setupAccel(lsm9ds1.LSM9DS1_ACCELRANGE_2G);
  lsm9ds1.setupMag(lsm9ds1.LSM9DS1_MAGGAIN_4GAUSS);
  lsm9ds1.setupGyro(lsm9ds1.LSM9DS1_GYROSCALE_245DPS);
}
