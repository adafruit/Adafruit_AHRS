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

#include "imuMaths.h"

using namespace imu;

double max(double a, double b)
{
    if(a >= b)
        return a;
    return b;
}

Vector::Vector()
{
    _x = _y = _z = 0;
}

Vector::Vector(double a, double b, double c)
{
    _x = a;
    _y = b;
    _z = c;
}

double Vector::magnitude()
{
    double res = (_x*_x) + (_y*_y) + (_z*_z);
    if((res > 1.001) || (res < 0.999))
        return sqrt(res);
    return 1;
}


void Vector::normalize()
{
    double mag = magnitude();
    if(mag != 0)
    {
        _x = _x/mag;
        _y = _y/mag;
        _z = _z/mag;
    }
}

Vector Vector::cross(Vector vec)
{
    Vector ret;
    ret._x = (_y * vec._z) - (_z * vec._y);
    ret._y = (_z * vec._x) - (_x * vec._z);
    ret._z = (_x * vec._y) - (_y * vec._x);
    return ret;
}

Vector Vector::scale(double scalar)
{
    Vector ret;
    ret.x() = _x * scalar;
    ret.y() = _y * scalar;
    ret.z() = _z * scalar;
    return ret;
}

void Vector::toDegrees()
{
    _x *= 180/M_PI;
    _y *= 180/M_PI;
    _z *= 180/M_PI;
}

void Vector::toRadians()
{
    _x *= M_PI/180;
    _y *= M_PI/180;
    _z *= M_PI/180;
}


double Vector::dot(Vector vec)
{
    return (_x * vec._x) + (_y * vec._y) + (_z * vec._z);
}

Vector Vector::operator + (Vector vec)
{
    Vector ret;

    ret.x() = _x + vec._x;
    ret.y() = _y + vec._y;
    ret.z() = _z + vec._z;

    return ret;
}

Vector Vector::operator - (Vector vec)
{
    Vector ret;

    ret.x() = _x - vec._x;
    ret.y() = _y - vec._y;
    ret.z() = _z - vec._z;

    return ret;
}

Vector Vector::operator * (double scalar)
{
    return scale(scalar);
}

Vector Vector::operator / (double scalar)
{
    Vector ret;
    ret.x() = _x / scalar;
    ret.y() = _y / scalar;
    ret.z() = _z / scalar;
    return ret;
}


Matrix::Matrix()
{
    uint8_t i;
    uint8_t j;
    for(i = 0; i <= 2; i++)
        for(j = 0; j <= 2; j++)
        {
            if(i == j)
                _cell[i][j] = 1.0;
            else
                _cell[i][j] = 0;
        }
}


Matrix Matrix::operator + (Matrix m)
{
    Matrix ret;

    uint8_t i;
    uint8_t j;
    for(i = 0; i <= 2; i++)
        for(j = 0; j <= 2; j++)
            ret._cell[i][j] = _cell[i][j] + m._cell[i][j];

    return ret;
}

Matrix Matrix::operator - (Matrix m)
{
    Matrix ret;

    uint8_t i;
    uint8_t j;
    for(i = 0; i <= 2; i++)
        for(j = 0; j <= 2; j++)
            ret._cell[i][j] = _cell[i][j] - m._cell[i][j];

    return ret;
}

Matrix Matrix::operator * (double scalar)
{
    Matrix ret;

    uint8_t i;
    uint8_t j;
    for(i = 0; i <= 2; i++)
        for(j = 0; j <= 2; j++)
            ret._cell[i][j] = _cell[i][j] * scalar;

    return ret;
}

Matrix Matrix::operator * (Matrix m)
{
    Matrix ret;

    uint8_t i;
    for(i = 0; i <= 2; i++)
    {
        uint8_t j;
        for(j = 0; j <= 2; j++)
        {
            //so for each cell in the output
            Vector row = row_to_vector(i);
            Vector column = m.col_to_vector(j);
            ret._cell[i][j] = row.dot(column);
        }
    }

    return ret;
}

Vector Matrix::col_to_vector(uint8_t rn)
{
    Vector ret;
    ret.x() = _cell[0][rn];
    ret.y() = _cell[1][rn];
    ret.z() = _cell[2][rn];
    return ret;
}

Vector Matrix::row_to_vector(uint8_t rn)
{
    Vector ret;
    ret.x() = _cell[rn][0];
    ret.y() = _cell[rn][1];
    ret.z() = _cell[rn][2];
    return ret;
}

void Matrix::vector_to_row(Vector v, int row)
{
    _cell[row][0] = v.x();
    _cell[row][1] = v.y();
    _cell[row][2] = v.z();
}

double& Matrix::operator ()(int row, int col)
{
    return _cell[row][col];
}

double& Matrix::cell(int row, int col)
{
    return _cell[row][col];
}


Quaternion::Quaternion()
{
    _w = 1.0;
    _x = _y = _z = 0.0;
}

Quaternion::Quaternion(double iw, double i_x, double i_y, double i_z)
{
    _w = iw;
    _x = i_x;
    _y = i_y;
    _z = i_z;
}

Quaternion::Quaternion(double iw, Vector vec)
{
    _w = iw;
    _x = vec.x();
    _y = vec.y();
    _z = vec.z();
}

double Quaternion::magnitude()
{
    double res = (_w*_w) + (_x*_x) + (_y*_y) + (_z*_z);
    if((res > 1.001) || (res < 0.999)) //only normalise if it needs to be done - saves a call to sqrt()
        return sqrt(res);
    return 1;
}

void Quaternion::normalize()
{
    double mag = magnitude();
    if(mag == 0) //avoid divide b_y _zero
        return;
    _w = _w/mag;
    _x = _x/mag;
    _y = _y/mag;
    _z = _z/mag;
}

Quaternion Quaternion::operator * (Quaternion q)
{
    Quaternion ret;
    ret._w = ((_w*q._w) - (_x*q._x) - (_y*q._y) - (_z*q._z));
    ret._x = ((_w*q._x) + (_x*q._w) + (_y*q._z) - (_z*q._y));
    ret._y = ((_w*q._y) - (_x*q._z) + (_y*q._w) + (_z*q._x));
    ret._z = ((_w*q._z) + (_x*q._y) - (_y*q._x) + (_z*q._w));
    return ret;
}


void Quaternion::fromMatrix(Matrix m)
{
    float tr = m(0, 0) + m(1, 1) + m(2, 2);

    float S = 0.0;
    if (tr > 0)
    {
        S = sqrt(tr+1.0) * 2;
        _w = 0.25 * S;
        _x = (m(2, 1) - m(1, 2)) / S;
        _y = (m(0, 2) - m(2, 0)) / S;
        _z = (m(1, 0) - m(0, 1)) / S;
    }
    else if ((m(0, 0) < m(1, 1))&(m(0, 0) < m(2, 2)))
    {
        S = sqrt(1.0 + m(0, 0) - m(1, 1) - m(2, 2)) * 2;
        _w = (m(2, 1) - m(1, 2)) / S;
        _x = 0.25 * S;
        _y = (m(0, 1) + m(1, 0)) / S;
        _z = (m(0, 2) + m(2, 0)) / S;
    }
    else if (m(1, 1) < m(2, 2))
    {
        S = sqrt(1.0 + m(1, 1) - m(0, 0) - m(2, 2)) * 2;
        _w = (m(0, 2) - m(2, 0)) / S;
        _x = (m(0, 1) + m(1, 0)) / S;
        _y = 0.25 * S;
        _z = (m(1, 2) + m(2, 1)) / S;
    }
    else
    {
        S = sqrt(1.0 + m(2, 2) - m(0, 0) - m(1, 1)) * 2;
        _w = (m(1, 0) - m(0, 1)) / S;
        _x = (m(0, 2) + m(2, 0)) / S;
        _y = (m(1, 2) + m(2, 1)) / S;
        _z = 0.25 * S;
    }
}

Matrix Quaternion::toMatrix()
{
    Matrix ret;
    ret.cell(0, 0) = 1-(2*(_y*_y))-(2*(_z*_z));
    ret.cell(0, 1) = (2*_x*_y)-(2*_w*_z);
    ret.cell(0, 2) = (2*_x*_z)+(2*_w*_y);

    ret.cell(1, 0) = (2*_x*_y)+(2*_w*_z);
    ret.cell(1, 1) = 1-(2*(_x*_x))-(2*(_z*_z));
    ret.cell(1, 2) = (2*(_y*_z))-(2*(_w*_x));

    ret.cell(2, 0) = (2*(_x*_z))-(2*_w*_y);
    ret.cell(2, 1) = (2*_y*_z)+(2*_w*_x);
    ret.cell(2, 2) = 1-(2*(_x*_x))-(2*(_y*_y));
    return ret;
}

Vector Quaternion::toEuler()
{
    Vector ret;
    double sqw = _w*_w;
    double sqx = _x*_x;
    double sqy = _y*_y;
    double sqz = _z*_z;

    ret.x() = atan2(2.0*(_x*_y+_z*_w),(sqx-sqy-sqz+sqw));
    ret.y() = asin(-2.0*(_x*_z-_y*_w)/(sqx+sqy+sqz+sqw));
    ret.z() = atan2(2.0*(_y*_z+_x*_w),(-sqx-sqy+sqz+sqw));

    return ret;
}

Quaternion Quaternion::conjugate()
{
    Quaternion q;
    q.w() = _w;
    q.x() = -_x;
    q.y() = -_y;
    q.z() = -_z;
    return q;
}


