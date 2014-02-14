/*
    AltIMU-10 Quaternion based AHRS
  	www.camelsoftware.com/firetail
    Copyright (C) 2013  Samuel Cowen

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*  Set the baud rate to 115200 to use this sketch, and adjust the
    output units by entering one of the following commands in the
    Serial Monitor and clicking the "send" button:    

    "output_euler"    - Euler angle output (default value)
    "output_mat"      - NED cosine matrix output
    "output_quat"     - Quaternion output
*/

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include "L3G.h"
#include "LSM303.h"
#include "imuMaths.h"

#define OUTPUT_EULER      0 // heading, pitch, roll
#define OUTPUT_MATRIX     1 // a north-east-down rotation matrix
#define OUTPUT_QUATERNION 2 // a quaternion!

L3G l3g;
LSM303 lsm303;
imu::Quaternion rotation_buf;
imu::Quaternion offset;

long delta_t;

bool warmup = true;
long start_millis;

//float alt = 0;
//float temp = 0;

int output_mode = OUTPUT_EULER;
bool auto_output = true;

int mag_cal_counter;

int write_offset(imu::Quaternion* value)
{
    const byte* p = (const byte*)value;
    int i;
    for (i = 0; i < sizeof(imu::Quaternion); i++) 
        EEPROM.write(i, *p++);
    return i;
}

int read_offset(imu::Quaternion* value)
{
    byte* p = (byte*)value;
    int i;
    for (i = 0; i < sizeof(imu::Quaternion); i++)
        *p++ = EEPROM.read(i);
    return i;
}


int write_mag_cal(imu::Vector min, imu::Vector max)
{
	byte* p = (byte*)&min;
	int i;
	int ret;
	int pos = sizeof(imu::Quaternion);
	for(i = pos; i < pos + sizeof(imu::Vector); i++)
            EEPROM.write(i, *p++);
	ret = i;

	p = (byte*)&max;
	pos = sizeof(imu::Quaternion) + sizeof(imu::Vector);
	for(i = pos; i < pos + sizeof(imu::Vector); i++)
            EEPROM.write(i, *p++);

	return i + ret;
}

int read_mag_cal(imu::Vector* min, imu::Vector* max)
{
	byte* p = (byte*)min;
	int i;
	int ret;
	int pos = sizeof(imu::Quaternion);
	for(i = pos; i < pos + sizeof(imu::Vector); i++)
            *p++ = EEPROM.read(i);
	ret = i;

	p = (byte*)&max;
	pos = sizeof(imu::Quaternion) + sizeof(imu::Vector);
	for(i = pos; i < pos + sizeof(imu::Vector); i++)
            *p++ = EEPROM.read(i);

	return i + ret;
}


void output(imu::Quaternion rotation)
{
	switch(output_mode)
	{
		case OUTPUT_EULER:
		{
			imu::Vector euler;
			euler = rotation.toEuler();
			euler.toDegrees();

			Serial.print(F("euler: "));
			Serial.print(euler.x());
			Serial.print(F(" "));
			Serial.print(euler.y());
			Serial.print(F(" "));
	    Serial.println(euler.z());
		}
		break;
		case OUTPUT_MATRIX:
		{
			imu::Matrix m = rotation.toMatrix();
			Serial.print(F("matrix: "));

			int i, j;
			for(i = 0; i <= 2; i++)
			{
				for(j = 0; j <= 2; j++)
				{
					Serial.print(m.cell(i, j)); Serial.print(" ");
				}
			}
			Serial.print(F("\n"));
		}
		break;
		case OUTPUT_QUATERNION:
		{
			Serial.print(F("quat: "));
			Serial.print(rotation.w());
			Serial.print(F(" "));
			Serial.print(rotation.x());
			Serial.print(F(" "));
			Serial.print(rotation.y());
			Serial.print(F(" "));
			Serial.println(rotation.z());
		}
		break;
	}

	//Serial.print("alt: ");
	//Serial.println(alt);
	//Serial.print("temp: ");
	//Serial.println(temp);
}


void setup()
{
    Serial.begin(115200);
    Serial.setTimeout(1);
    Serial.println(F("Adafruit 9/10DOF AHRS"));
    Serial.println(F("www.CamelSoftware.com"));
    delta_t = millis();

    Wire.begin();
    l3g.init();
    l3g.enableDefault();
    l3g.measureOffsets();
    lsm303.init();
    lsm303.enableDefault();
    //lps331.init();
    //lps331.enableDefault();
    Serial.println(F("Done"));

    mag_cal_counter = 0;

    start_millis = millis();
    //read_offset(&offset);

    if(abs(offset.magnitude() - 1) >= 0.1 || isnan(offset.magnitude())) //if installation offset isn't a valid unit quaternion
    {
      imu::Quaternion qz; //set it to a unit quaternion with no rotation (no offset)
      offset = qz;
    }

    //read_mag_cal(&lsm303.m_min, &lsm303.m_max);

    imu::Vector acc;
    imu::Vector mag;

    lsm303.readAcc();
    lsm303.readMag();

    acc = lsm303.a;
    mag = lsm303.m;

    imu::Vector down = acc.invert();
    imu::Vector east = down.cross(mag);
    imu::Vector north = east.cross(down);

    down.normalize();
    east.normalize();
    north.normalize();

    imu::Matrix m;
    m.vector_to_row(north, 0);
    m.vector_to_row(east, 1);
    m.vector_to_row(down, 2);

    rotation_buf.fromMatrix(m);
}

void loop()
{
    double dt = millis() - delta_t;
    if(dt < 20)
        return;

	dt /= 1000;
    if(warmup)
    {
        if((millis() - start_millis) > 200)
        {
            warmup = false;
        }
    }

    delta_t = millis();

    imu::Vector ang_vel;
    imu::Vector acc;
    imu::Vector mag;

    l3g.read();

    ang_vel = l3g.g;

    lsm303.readAcc();
    lsm303.readMag();

    acc = lsm303.a;
    mag = lsm303.m;

	if(mag_cal_counter) //if a mag cal is underway
	{
		mag_cal_counter++; //increment counter
		if(mag_cal_counter > 3000) //if counter has reached 3000, mag cal has been running for 60 seconds
		{
			mag_cal_counter = 0; //stop the mag cal
			write_mag_cal(lsm303.m_min, lsm303.m_max);
		}

		if(mag.x() > lsm303.m_max.x())
			lsm303.m_max.x() = mag.x();
		if(mag.y() > lsm303.m_max.y())
			lsm303.m_max.y() = mag.y();
		if(mag.z() > lsm303.m_max.z())
			lsm303.m_max.z() = mag.z();

		if(mag.x() < lsm303.m_min.x())
			lsm303.m_max.x() = mag.x();
		if(mag.y() < lsm303.m_min.y())
			lsm303.m_max.y() = mag.y();
		if(mag.z() < lsm303.m_min.z())
			lsm303.m_max.z() = mag.z();
	}


    imu::Vector correction;

    if(abs(acc.magnitude()-1) <= 0.15)
    {
        float correction_strength = 0.0005;
		if(warmup)
			correction_strength = 0.01;

        imu::Vector down = acc.invert();
        imu::Vector east = down.cross(mag);
        imu::Vector north = east.cross(down);

        down.normalize();
        east.normalize();
        north.normalize();

        imu::Matrix rotationMatrix = rotation_buf.toMatrix();
        correction = (
                         north.cross(rotationMatrix.row_to_vector(0)) +
                         east.cross(rotationMatrix.row_to_vector(1)) +
                         down.cross(rotationMatrix.row_to_vector(2))
                     ) * correction_strength;
    }

	imu::Vector w;
	w = ang_vel + correction;

    imu::Quaternion q(1, w.x()*dt/2.0, w.y()*dt/2.0, w.z()*dt/2.0);

    rotation_buf = rotation_buf*q;
    rotation_buf.normalize();
	
	imu::Quaternion rotation = offset.conjugate()*rotation_buf;
	rotation.normalize();

	//alt = (0.1 * lps331.pressureToAltitudeFeet(lps331.readPressureInchesHg())) + (0.9) * alt;
	//temp = (0.1 * lps331.readTemperatureC()) + (0.9) * temp;

	if(auto_output)
		output(rotation);


	char buffer[20];
	memset(buffer, '\0', 20);
	Serial.readBytesUntil('\n', buffer, 20);

	if(strncmp(buffer, "output_euler", 12) == 0)
		output_mode= OUTPUT_EULER;
	
	if(strncmp(buffer, "output_mat", 11) == 0)
		output_mode = OUTPUT_MATRIX;

	if(strncmp(buffer, "output_quat", 12) == 0)
		output_mode = OUTPUT_QUATERNION;

	if(strncmp(buffer, "auto_on", 7) == 0)
		auto_output = true;

	if(strncmp(buffer, "auto_off", 8) == 0)
		auto_output = false;

	if(strncmp(buffer, "read", 4) == 0)
		output(rotation);


	if(strncmp(buffer, "set_offset", 6) == 0)
	{
		offset = rotation_buf;
		write_offset(&offset);
	}
	if(strncmp(buffer, "mag_cal", 7) == 0)
	{
		mag_cal_counter = 1;
		lsm303.m_max = imu::Vector();
		lsm303.m_min = imu::Vector();
	}

}
