#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_10DOF.h>
#include <Adafruit_Simple_AHRS.h>

Adafruit_10DOF board;
Adafruit_Simple_AHRS ahrs(board);

// Update this with the correct SLP for accurate altitude measurements
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

void setup()
{
  Serial.begin(115200);
  Serial.println(F("Adafruit 10 DOF Pitch/Roll/Heading Example")); Serial.println("");
  
  // Initialize the board.
  board.begin();
}

void loop(void)
{
  sensors_vec_t   orientation;

  // Use the simple AHRS function to get the current orientation.
  if (ahrs.getOrientation(&orientation))
  {
    /* 'orientation' should have valid .roll and .pitch fields */
    Serial.print(F("Orientation: "));
    Serial.print(orientation.roll);
    Serial.print(F(" "));
    Serial.print(orientation.pitch);
    Serial.print(F(" "));
    Serial.print(orientation.heading);
    Serial.println(F(""));
  }

  // Calculate the altitude using the barometric pressure sensor
  sensors_event_t bmp_event;
  board.getBMP().getEvent(&bmp_event);
  if (bmp_event.pressure)
  {
    /* Get ambient temperature in C */
    float temperature;
    board.getBMP().getTemperature(&temperature);
    /* Convert atmospheric pressure, SLP and temp to altitude */
    Serial.print(F("Alt: "));
    Serial.print(board.getBMP().pressureToAltitude(seaLevelPressure,
                                        bmp_event.pressure,
                                        temperature)); 
    Serial.println(F(""));
    /* Display the temperature */
    Serial.print(F("Temp: "));
    Serial.print(temperature);
    Serial.println(F(""));
  }
  
  delay(100);
}
