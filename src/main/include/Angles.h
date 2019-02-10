#include "math.h"
struct Radian;
struct Degree;

struct Radian{
    double value;
    Radian(double value){
        this->value=value;
    }
    explicit Radian(Degree value){
        this->value=value.value/360*2*M_PI;
    }
    Radian operator+(Radian summer){
        return Radian(this->value+summer.value);
    }
    Radian operator+(Degree summer){
        return Radian(this->value+summer.value*2* M_PI / 360.0);
    }
    Radian operator-(Radian summer){
        return Radian(this->value-summer.value);
    }
    Radian operator-(Degree summer){
        return Radian(this->value-summer.value*2* M_PI / 360.0);
    }
}
struct Degree{
    double value;
    Degree(double value){
        this->value=value;
    }
    explicit Degree(Radian value){
        this->value=value.value*360/2/M_PI;
    }
    Degree operator+(Radian summer){
        return Degree(this->value+summer.value*360/2/M_PI);
    }
    Degree operator+(Degree summer){
        return Degree(this->value+summer.value);
    }
    Degree operator-(Radian summer){
        return Degree(this->value+summer.value*360/2/M_PI);
    }
    Degree operator-(Degree summer){
        return Degree(this->value+summer.value);
    }
}