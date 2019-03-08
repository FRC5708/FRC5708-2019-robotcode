#include "subsystems/ScootyPooty.h"
#include "Robot.h"
#include <iostream>
#include <math.h>


ScootyPooty::ScootyPooty() : frc::Subsystem("ScootyPooty") {
	
	leftEncoder->SetDistancePerPulse(1.0/360.0);
	rightEncoder->SetDistancePerPulse(1.0/360.0);
	
	BLMotor->SetInverted(true);
	FLMotor->SetInverted(true);
}

double ScootyPooty::Limit(double number) {
  if (number > 1.0) {
    return 1.0;
  }
  if (number < -1.0) {
    return -1.0;
  }
  return number;
}

void ScootyPooty::Drive(double left, double right) {
	FLMotor->Set(left);
	BLMotor->Set(left);
	FRMotor->Set(right);
	BRMotor->Set(right);
}

void ScootyPooty::DrivePolar(double moveValue, double rotateValue) {

  double leftMotorOutput;
  double rightMotorOutput;

  moveValue = Limit(moveValue);
  rotateValue = Limit(rotateValue);


  if (moveValue > 0.0) {
    if (rotateValue > 0.0) {
      leftMotorOutput = moveValue - rotateValue;
      rightMotorOutput = std::max(moveValue, rotateValue);
    } else {
      leftMotorOutput = std::max(moveValue, -rotateValue);
      rightMotorOutput = moveValue + rotateValue;
    }
  } else {
    if (rotateValue > 0.0) {
      leftMotorOutput = -std::max(-moveValue, rotateValue);
      rightMotorOutput = moveValue + rotateValue;
    } else {
      leftMotorOutput = moveValue - rotateValue;
      rightMotorOutput = -std::max(-moveValue, -rotateValue);
    }
  }
  Drive(leftMotorOutput,rightMotorOutput);
}

void ScootyPooty::ResetDistance(){
	leftEncoder->Reset();
	rightEncoder->Reset();

	leftEncoder->SetDistancePerPulse(1.0/360.0);
	rightEncoder->SetDistancePerPulse(1.0/360.0);
}



