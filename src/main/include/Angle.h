/*
 * Angle.h
 *
 *  Created on: Feb 10, 2019
 *      Author: plotner
 */

#ifndef ANGLE_H_
#define ANGLE_H_
#include "math.h"
struct Radian;
struct Degree;
struct Radian{
	double value;
	Radian(double);
	Radian(Degree);
	Radian();
	Radian operator +(Radian);
	Radian operator +(Degree);
	Radian operator -(Radian);
	Radian operator -(Degree);
	Radian operator *(double);
	Radian operator /(double);
	operator double();
};
struct Degree{
	double value;
	Degree(double);
	Degree(Radian);
	Degree();
	Degree operator +(Radian);
	Degree operator +(Degree);
	Degree operator -(Radian);
	Degree operator -(Degree);
	Degree operator *(double);
	Degree operator /(double);
	operator double();
};


#endif /* ANGLE_H_ */
