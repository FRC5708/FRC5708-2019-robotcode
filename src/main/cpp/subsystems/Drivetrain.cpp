#include "subsystems/Drivetrain.h"
#include "Robot.h"
#include <iostream>
#include <math.h>


Drivetrain::Drivetrain() : frc::Subsystem("Drivetrain") {
	
	leftEncoder->SetDistancePerPulse(1.0/360.0);
	rightEncoder->SetDistancePerPulse(1.0/360.0);
	
	BLMotor->SetInverted(true);
	FLMotor->SetInverted(true);
}

double Drivetrain::Limit(double number) {
	if (number > 1.0) {
		return 1.0;
	}
	if (number < -1.0) {
		return -1.0;
	}
	return number;
}

void Drivetrain::Drive(double left, double right) {
	FLMotor->Set(left);
	BLMotor->Set(left);
	FRMotor->Set(right);
	BRMotor->Set(right);
}

void Drivetrain::DrivePolar(double moveValue, double rotateValue) {

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

void Drivetrain::ResetDistance(){
	leftEncoder->Reset();
	rightEncoder->Reset();

	leftEncoder->SetDistancePerPulse(1.0/360.0);
	rightEncoder->SetDistancePerPulse(1.0/360.0);
}
constexpr double ROBOT_WIDTH = 27; // inches

double Drivetrain::GetDistance() {
	double leftDistance = leftEncoder->GetDistance(), rightDistance = rightEncoder->GetDistance();
	if (fabs(leftDistance) < 0.01) {
		// emulate with gyro
		return rightDistance * WheelCircumference + 
		Radian(Degree(Robot::gyro->GetAngle())) * (ROBOT_WIDTH / 2.0);
	}
	else if (fabs(rightDistance) < 0.01) {
		return leftDistance * WheelCircumference - 
		Radian(Degree(Robot::gyro->GetAngle())) * (ROBOT_WIDTH / 2.0);
	}
	else {
		// both encoders, yay!
		return (leftDistance + rightDistance)/2.0 * WheelCircumference;
	}
}
double Drivetrain::GetRate() {
	double leftDistance = leftEncoder->GetDistance(), rightDistance = rightEncoder->GetDistance();
	if (fabs(leftDistance) < 0.01) {
		// emulate with gyro
		return leftEncoder->GetRate() * WheelCircumference
		 + Radian(Degree(Robot::gyro->GetRate())) * (ROBOT_WIDTH / 2.0);
	}
	else if (fabs(rightDistance) < 0.01) {
		return rightEncoder->GetRate() * WheelCircumference
		- Radian(Degree(Robot::gyro->GetRate())) * (ROBOT_WIDTH / 2.0);
	}
	else {
		// both encoders, yay!
		return (leftEncoder->GetRate() + rightEncoder->GetRate())/2.0 * WheelCircumference;
	}
}
Degree Drivetrain::GetGyroAngle() {
	Degree gyroAngle = Robot::gyro->GetAngle();
	if (fabs(gyroAngle) < 0.01) {

		// emulate with encoders
		return (leftEncoder->GetDistance() - rightEncoder->GetDistance())
		* WheelCircumference / ROBOT_WIDTH / M_PI * 180;
	}
	else return gyroAngle;
}
Degree Drivetrain::GetGyroRate() {
	Degree gyroAngle = Robot::gyro->GetAngle();
	if (fabs(gyroAngle) < 0.01) {

		// emulate with encoders
		return (leftEncoder->GetRate() - rightEncoder->GetRate())
		* WheelCircumference / ROBOT_WIDTH / M_PI * 180;
	}
	else return Robot::gyro->GetRate();
}