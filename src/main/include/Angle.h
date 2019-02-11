
#ifndef ANGLE_H_
#define ANGLE_H_
#include "math.h"
#include <type_traits>

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

template<class T> Radian Radian::operator *(T multer){
	static_assert(std::is_arithmetic<T>::value ,"Multiplication must be done with arithmetic type.");
    return Radian(this->value * multer);
}
template<class T> Radian Radian::operator /(T multer){
	static_assert(std::is_arithmetic<T>::value ,"Division must be done with arithmetic type.");
    return Radian(this->value / multer);
}

template<class T> Degree Degree::operator *(T multer){
	static_assert(std::is_arithmetic<T>::value ,"Multiplication must be done with arithmetic type.");
    return Degree(this->value * multer);
}
template<class T> Degree Degree::operator /(T multer){
	static_assert(std::is_arithmetic<T>::value ,"Division must be done with arithmetic type.");
    return Degree(this->value / multer);
}


#endif /* ANGLE_H_ */
