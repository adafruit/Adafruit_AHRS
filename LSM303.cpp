#include "LSM303.h"
#include <Wire.h>
#include <math.h>

// Defines ////////////////////////////////////////////////////////////////

// The Arduino two-wire interface uses a 7-bit number for the address,
// and sets the last bit correctly based on reads and writes
#define MAG_ADDRESS            (0x3C >> 1)
#define ACC_ADDRESS_SA0_A_LOW  (0x30 >> 1)
#define ACC_ADDRESS_SA0_A_HIGH (0x32 >> 1)

// Constructors ////////////////////////////////////////////////////////////////

LSM303::LSM303(void)
{
    // These are just some values for a particular unit; it is recommended that
    // a calibration be done for your particular unit.
    m_max.x() = +540;
    m_max.y() = +500;
    m_max.z() = 180;

    m_min.x() = -520;
    m_min.y() = -570;
    m_min.z() = -770;

    _device = LSM303_DEVICE_AUTO;
    acc_address = ACC_ADDRESS_SA0_A_LOW;

    io_timeout = 0;  // 0 = no timeout
    did_timeout = false;
}

// Public Methods //////////////////////////////////////////////////////////////

bool LSM303::timeoutOccurred()
{
    return did_timeout;
}

void LSM303::setTimeout(unsigned int timeout)
{
    io_timeout = timeout;
}

unsigned int LSM303::getTimeout()
{
    return io_timeout;
}

void LSM303::init(byte device, byte sa0_a)
{
    _device = device;
    switch (_device)
    {
    case LSM303DLH_DEVICE:
    case LSM303DLM_DEVICE:
        if (sa0_a == LSM303_SA0_A_LOW)
            acc_address = ACC_ADDRESS_SA0_A_LOW;
        else if (sa0_a == LSM303_SA0_A_HIGH)
            acc_address = ACC_ADDRESS_SA0_A_HIGH;
        else
            acc_address = (detectSA0_A() == LSM303_SA0_A_HIGH) ? ACC_ADDRESS_SA0_A_HIGH : ACC_ADDRESS_SA0_A_LOW;
        break;

    case LSM303DLHC_DEVICE:
        acc_address = ACC_ADDRESS_SA0_A_HIGH;
        break;

    default:
        // try to auto-detect device
        if (detectSA0_A() == LSM303_SA0_A_HIGH)
        {
            // if device responds on 0011001b (SA0_A is high), assume DLHC
            acc_address = ACC_ADDRESS_SA0_A_HIGH;
            _device = LSM303DLHC_DEVICE;
        }
        else
        {
            // otherwise, assume DLH or DLM (pulled low by default on Pololu boards); query magnetometer WHO_AM_I to differentiate these two
            acc_address = ACC_ADDRESS_SA0_A_LOW;
            _device = (readMagReg(LSM303_WHO_AM_I_M) == 0x3C) ? LSM303DLM_DEVICE : LSM303DLH_DEVICE;
        }
    }
}

// Turns on the LSM303's accelerometer and magnetometers and places them in normal
// mode.
void LSM303::enableDefault(void)
{
    if (_device == LSM303DLHC_DEVICE)
    {
        writeAccReg(LSM303_CTRL_REG1_A, 0b01000111); // Normal power mode, all axes enabled, 50 Hz
        writeAccReg(LSM303_CTRL_REG4_A, 0x28); // 8 g full scale: FS = 10 on DLHC, high resolution output mode
    }
    else
    {
        writeAccReg(LSM303_CTRL_REG1_A, 0b00100111); // normal power mode, all axes enabled, 50 Hz
        writeAccReg(LSM303_CTRL_REG4_A, 0b00110000); // 8 g full scale: FS = 11 on DLH, DLM
    }

    // Enable Magnetometer
    // 0x00 = 0b00000000
    // Continuous conversion mode
    writeMagReg(LSM303_MR_REG_M, 0x00);
}

// Writes an accelerometer register
void LSM303::writeAccReg(byte reg, byte value)
{
    Wire.beginTransmission(acc_address);
    Wire.write(reg);
    Wire.write(value);
    last_status = Wire.endTransmission();
}

// Reads an accelerometer register
byte LSM303::readAccReg(byte reg)
{
    byte value;

    Wire.beginTransmission(acc_address);
    Wire.write(reg);
    last_status = Wire.endTransmission();
    Wire.requestFrom(acc_address, (byte)1);
    value = Wire.read();
    Wire.endTransmission();

    return value;
}

// Writes a magnetometer register
void LSM303::writeMagReg(byte reg, byte value)
{
    Wire.beginTransmission(MAG_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    last_status = Wire.endTransmission();
}

// Reads a magnetometer register
byte LSM303::readMagReg(int reg)
{
    byte value;

    // if dummy register address (magnetometer Y/Z), use device type to determine actual address
    if (reg < 0)
    {
        switch (reg)
        {
        case LSM303_OUT_Y_H_M:
            reg = (_device == LSM303DLH_DEVICE) ? LSM303DLH_OUT_Y_H_M : LSM303DLM_OUT_Y_H_M;
            break;
        case LSM303_OUT_Y_L_M:
            reg = (_device == LSM303DLH_DEVICE) ? LSM303DLH_OUT_Y_L_M : LSM303DLM_OUT_Y_L_M;
            break;
        case LSM303_OUT_Z_H_M:
            reg = (_device == LSM303DLH_DEVICE) ? LSM303DLH_OUT_Z_H_M : LSM303DLM_OUT_Z_H_M;
            break;
        case LSM303_OUT_Z_L_M:
            reg = (_device == LSM303DLH_DEVICE) ? LSM303DLH_OUT_Z_L_M : LSM303DLM_OUT_Z_L_M;
            break;
        }
    }

    Wire.beginTransmission(MAG_ADDRESS);
    Wire.write(reg);
    last_status = Wire.endTransmission();
    Wire.requestFrom(MAG_ADDRESS, 1);
    value = Wire.read();
    Wire.endTransmission();

    return value;
}

void LSM303::setMagGain(magGain value)
{
    Wire.beginTransmission(MAG_ADDRESS);
    Wire.write(LSM303_CRB_REG_M);
    Wire.write((byte) value);
    Wire.endTransmission();
}

// Reads the 3 accelerometer channels and stores them in vector a
void LSM303::readAcc(void)
{
    Wire.beginTransmission(acc_address);
    // assert the MSB of the address to get the accelerometer
    // to do slave-transmit subaddress updating.
    Wire.write(LSM303_OUT_X_L_A | (1 << 7));
    last_status = Wire.endTransmission();
    Wire.requestFrom(acc_address, (byte)6);

    unsigned int millis_start = millis();
    did_timeout = false;
    while (Wire.available() < 6) {
        if (io_timeout > 0 && ((unsigned int)millis() - millis_start) > io_timeout) {
            did_timeout = true;
            return;
        }
    }

    byte xla = Wire.read();
    byte xha = Wire.read();
    byte yla = Wire.read();
    byte yha = Wire.read();
    byte zla = Wire.read();
    byte zha = Wire.read();

    // combine high and low bytes, then shift right to discard lowest 4 bits (which are meaningless)
    // GCC performs an arithmetic right shift for signed negative numbers, but this code will not work
    // if you port it to a compiler that does a logical right shift instead.
    a.x() = ((int16_t)(xha << 8 | xla)) >> 4;
    a.y() = ((int16_t)(yha << 8 | yla)) >> 4;
    a.z() = ((int16_t)(zha << 8 | zla)) >> 4;

    const float accel_scale = 0.0039;
    a = a*accel_scale;
}

// Reads the 3 magnetometer channels and stores them in vector m
void LSM303::readMag(void)
{
    Wire.beginTransmission(MAG_ADDRESS);
    Wire.write(LSM303_OUT_X_H_M);
    last_status = Wire.endTransmission();
    Wire.requestFrom(MAG_ADDRESS, 6);

    unsigned int millis_start = millis();
    did_timeout = false;
    while (Wire.available() < 6) {
        if (io_timeout > 0 && ((unsigned int)millis() - millis_start) > io_timeout) {
            did_timeout = true;
            return;
        }
    }

    byte xhm = Wire.read();
    byte xlm = Wire.read();

    byte yhm, ylm, zhm, zlm;


    // DLM, DLHC: register address for Z comes before Y
    zhm = Wire.read();
    zlm = Wire.read();
    yhm = Wire.read();
    ylm = Wire.read();


    // combine high and low bytes
    m.x() = (int16_t)(xhm << 8 | xlm);
    m.y() = (int16_t)(yhm << 8 | ylm);
    m.z() = (int16_t)(zhm << 8 | zlm);
}

// Reads all 6 channels of the LSM303 and stores them in the object variables
void LSM303::read(void)
{
    readAcc();
    readMag();
}

// Private Methods //////////////////////////////////////////////////////////////

byte LSM303::detectSA0_A(void)
{
    Wire.beginTransmission(ACC_ADDRESS_SA0_A_LOW);
    Wire.write(LSM303_CTRL_REG1_A);
    last_status = Wire.endTransmission();
    Wire.requestFrom(ACC_ADDRESS_SA0_A_LOW, 1);
    if (Wire.available())
    {
        Wire.read();
        return LSM303_SA0_A_LOW;
    }
    else
        return LSM303_SA0_A_HIGH;
}
