# Adafruit AHRS library

This library lets you take an accelerometer, gyroscope and magnetometer and 
combine the data to create orientation data.

Options are Mahony (lowest memory/computation), Madgwick (fair memory/computation) and NXP fusion/Kalman (highest).

While in theory these can run on an Arduino UNO/Atmega328P we really recommend a SAMD21 or better. Having single-instruction floating point multiply and plenty of RAM will help a lot!