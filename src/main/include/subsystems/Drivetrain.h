#ifndef Drivetrain_H
#define Drivetrain_H

#include <frc/commands/Subsystem.h>
#include <frc/Spark.h>
#include <frc/Encoder.h>
#include <vector>
#include "RobotMap.h"
#include <cmath>
#include "Angle.h"


class Drivetrain : public frc::Subsystem {
public:
	Drivetrain();
	void InitDefaultCommand() override {};
	double Limit(double number);
	void Drive(double left, double right);				//Drives left and right wheels accordingly
	void DrivePolar(double moveValue, double rotateValue);	//Drives at moveValue and rotateValue
	void Periodic() override;
	void ResetDistance();
	double GetDistance();
	double GetRate();
	Degree GetGyroAngle();
	Degree GetGyroRate();

	frc::Encoder* leftEncoder = new frc::Encoder(LeftEncoderChannel[0],LeftEncoderChannel[1], true);
	frc::Encoder* rightEncoder = new frc::Encoder(RightEncoderChannel[0],RightEncoderChannel[1], false);
private:

	frc::SpeedController* FLMotor = new frc::Spark(FLMotorChannel);
	frc::SpeedController* BLMotor = new frc::Spark(BLMotorChannel);
	frc::SpeedController* FRMotor = new frc::Spark(FRMotorChannel);
	frc::SpeedController* BRMotor = new frc::Spark(BRMotorChannel);

	int ticksSinceLastDrive = 0;
};


#endif  // Drivetrain_H
