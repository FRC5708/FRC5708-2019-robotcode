
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
	template<class T>
	Radian operator *(T);
	template<class T>
	Radian operator /(T);
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
	template<class T>
	Degree operator *(T);
	template<class T>
	Degree operator /(T);
	operator double();
};


#endif /* ANGLE_H_ */
