#include "subsystems/Drivetrain.h"
#include "Robot.h"
#include <frc/DriverStation.h>
#include <frc/Spark.h>
#include <frc/PWMTalonSRX.h>
#include <iostream>
#include <math.h>


Drivetrain::Drivetrain() : frc::Subsystem("Drivetrain") {
	
	if (IS_PROD) {
		FLMotor = new frc::Spark(FLMotorChannel);
		FRMotor = new frc::Spark(FRMotorChannel);
		BLMotor = new frc::Spark(BLMotorChannel);
		BRMotor = new frc::Spark(BRMotorChannel);
	}
	else {
		FLMotor = new frc::PWMTalonSRX(FLMotorChannel);
		FRMotor = new frc::PWMTalonSRX(FRMotorChannel);
		BLMotor = new frc::PWMTalonSRX(BLMotorChannel);
		BRMotor = new frc::PWMTalonSRX(BRMotorChannel);
	}

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

void Drivetrain::Periodic() {
	if (ticksSinceLastDrive >= 2) {
		Drive(0, 0);
		std::cout << "Drivetrain has not been called, and has been reset. This is a BUG." << std::endl;
	}
	if (frc::DriverStation::GetInstance().IsEnabled()) {
		++ticksSinceLastDrive;
	}
}

void Drivetrain::Drive(double left, double right) {
	/*if (left == 0 && right == 0) {
		FLMotor->StopMotor();
		BLMotor->StopMotor();
		FRMotor->StopMotor();
		BRMotor->StopMotor();
	}
	else {*/
		FLMotor->Set(left);
		BLMotor->Set(left);
		FRMotor->Set(right);
		BRMotor->Set(right);
	//}

	ticksSinceLastDrive = 0;
}

void Drivetrain::DrivePolar(double moveValue, double rotateValue) {

	moveValue = Limit(moveValue);
	rotateValue = Limit(rotateValue);
	/*
	double leftMotorOutput;
	double rightMotorOutput;

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
	}*/

	double v = (1-fabs(rotateValue)) * (moveValue) + moveValue;
	double w = (1-fabs(moveValue)) * (rotateValue) + rotateValue;
	double rightMotorOutput = (v+w)/2;
	double leftMotorOutput = (v-w)/2;

	Drive(leftMotorOutput,rightMotorOutput);
}

void Drivetrain::ResetDistance(){
	leftEncoder->Reset();
	rightEncoder->Reset();

	leftEncoder->SetDistancePerPulse(1.0/360.0);
	rightEncoder->SetDistancePerPulse(1.0/360.0);
}

void Drivetrain::checkEncoders() {
	if (!leftEncoderGood) leftEncoderGood = fabs(leftEncoder->GetDistance()) > 0.1;
	if (!rightEncoderGood) rightEncoderGood = fabs(rightEncoder->GetDistance()) > 0.1;
}

double Drivetrain::GetDistance() {
	checkEncoders();
	double leftDistance = leftEncoder->GetDistance(), rightDistance = rightEncoder->GetDistance();
	
	if (!leftEncoderGood && rightEncoderGood) {
		// emulate with gyro
		return rightDistance * WheelCircumference + 
		Radian(Degree(Robot::gyro->GetAngle())) * (ROBOT_WIDTH / 2.0);
	}
	else if (!rightEncoderGood && leftEncoderGood) {
		return leftDistance * WheelCircumference - 
		Radian(Degree(Robot::gyro->GetAngle())) * (ROBOT_WIDTH / 2.0);
	}
	else {
		// both encoders, yay!
		// or maybe no encoders
		return (leftDistance + rightDistance)/2.0 * WheelCircumference;
	}
}
double Drivetrain::GetRate() {
	checkEncoders();

	if (!leftEncoderGood && rightEncoderGood) {
		// emulate with gyro
		return leftEncoder->GetRate() * WheelCircumference
		 + Radian(Degree(Robot::gyro->GetRate())) * (ROBOT_WIDTH / 2.0);
	}
	else if (!rightEncoderGood && leftEncoderGood) {
		return rightEncoder->GetRate() * WheelCircumference
		- Radian(Degree(Robot::gyro->GetRate())) * (ROBOT_WIDTH / 2.0);
	}
	else {
		// both encoders, yay!
		// or maybe no encoders
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