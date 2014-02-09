/*
    AltIMU-10 Quaternion based AHRS
  	www.camelsoftware.com/firetail
    Copyright (C) 2013  Samuel Cowen

	  The latest version of the IMU Maths library will always be available at
	  http://sourceforge.net/p/firetail/code/ci/master/tree/

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

#ifndef IMUMATH_H
#define IMUMATH_H

#include <math.h>
#include <stdint.h>

namespace imu
{

class Vector
{
public:
    Vector();
    Vector(double _x, double _y, double _z);

    double magnitude(); //return the magnitude of the vector
    void normalize(); //normalise the vector

    Vector cross(Vector vec);
    double dot(Vector vec);

    Vector scale(double scalar);

    void toDegrees();
    void toRadians();

    double &x() {
        return _x;
    }
    double &y() {
        return _y;
    }
    double &z() {
        return _z;
    }

    Vector operator + (Vector);
    Vector operator - (Vector);
    Vector invert() {
        return Vector(-_x, -_y, -_z);
    }

    Vector operator * (double scalar);
	Vector operator / (double scalar);

private:
    double _x, _y, _z;
};


class Matrix
{
public:
    Matrix();

    Matrix operator + (Matrix);
    Matrix operator - (Matrix);
    Matrix operator * (double scalar);
    Matrix operator * (Matrix m);

    double& operator ()(int row, int col);
    double& cell(int row, int col);

    Vector row_to_vector(uint8_t rn);
    Vector col_to_vector(uint8_t cn);

    void vector_to_row(Vector v, int row);


protected:
    double _cell[3][3]; //x, y
};


class Quaternion
{
public:
    Quaternion();
    Quaternion(double iw, double ix, double iy, double iz);
    Quaternion(double w, Vector vec);

    double& w() {
        return _w;
    }
    double& x() {
        return _x;
    }
    double& y() {
        return _y;
    }
    double& z() {
        return _z;
    }

    double magnitude();
    void normalize();
    Quaternion conjugate();

    void fromMatrix(Matrix m);
    Matrix toMatrix();
    Vector toEuler();

    Quaternion operator * (Quaternion q);

private:
    double _w, _x, _y, _z;
};


};

#endif

