#include "math.h"
#include "Angle.h"

Radian::Radian(double value){
	this->value=value;
}
Radian::Radian(Degree value){
	this->value=value.value/360*2*M_PI;
}
Radian::Radian(){

}
Radian Radian::operator +(Radian summer){
	return Radian(this->value+summer.value);
}
Radian Radian::operator +(Degree summer){
	return Radian(this->value+summer.value*2* M_PI / 360.0);
}
Radian Radian::operator -(Radian summer){
	return Radian(this->value-summer.value);
}
Radian Radian::operator -(Degree summer){
	return Radian(this->value-summer.value*2* M_PI / 360.0);
}
Radian::operator double(){
	return this->value;
}


Degree::Degree(double value){
	this->value=value;
}
Degree::Degree(Radian value){
	this->value=value.value*360/2/M_PI;
}
Degree::Degree(){

}
Degree Degree::operator +(Radian summer){
	return Degree(this->value+summer.value*360/2/M_PI);
}
Degree Degree::operator +(Degree summer){
	return Degree(this->value+summer.value);
}
Degree Degree::operator -(Radian summer){
	return Degree(this->value+summer.value*360/2/M_PI);
}
Degree Degree::operator -(Degree summer){
	return Degree(this->value+summer.value);
}
Degree::operator double(){
	return this->value;
}