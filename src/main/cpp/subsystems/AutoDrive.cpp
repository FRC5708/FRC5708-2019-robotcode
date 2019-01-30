#include "subsystems/AutoDrive.h"
#include "Robot.h"

#include <algorithm>

AutoDrive::AutoDrive() : frc::Subsystem("AutoDrive"), 
currentPosition{ 0, 0, 0, 0 } {}

constexpr double maxCentripetal = 6*12, // in/sec^2
 topSpeed = 10*12, // estimated top speed of the robot at full power, inches/sec
 reachTopSpeed = 1, // estimated seconds to reach top speed from stop
 maxPower = 0.75, // maximum power the auton will run the motors at
 minForwardPower = 0.25; // below this the robot won't move


// NOTE: assumes gyro is clockwise=positive
void AutoDrive::Periodic() {
	updatePosition();

	constexpr double kTurning = 0.05,
	 kTurnReduction = 0.5,
	 kForwardPower = 0.05;

	Point curveAimOffset;
	if (target.isAngled) {

		double targetDistance = sqrt(pow(target.point.x - currentPosition.point.x, 2) + 
		pow(target.point.y - currentPosition.point.y, 2));

		double expectedSpeed = targetDistance / reachTopSpeed;
		if (target.slowDown) expectedSpeed /= 2;
		expectedSpeed = std::max(Robot::drivetrain.GetRate(), std::min(expectedSpeed, topSpeed));

		// allow for a turn with 3/4 of the maxCentripetal
		double turningRadius = pow(expectedSpeed, 2) / (maxCentripetal * 0.75);

		double targetAngle = target.angle/180*M_PI;
		double sideRobotAngle = (Robot::gyro->GetAngle() - 90)/180*M_PI;

		curveAimOffset.x = turningRadius * (sin(targetAngle) - sin(sideRobotAngle));
		curveAimOffset.y = turningRadius * (cos(targetAngle) - cos(sideRobotAngle));
	}
	else curveAimOffset = { 0, 0 };

	double pointAngle;
	double maxTurnPower;
	// if we haven't passed the aim point
	if (fabs(target.point.x - currentPosition.point.x) > fabs(curveAimOffset.x) &&
		fabs(target.point.y - currentPosition.point.y) > fabs(curveAimOffset.y)) {
		// Point towards the curve aim point
		pointAngle = atan2(target.point.x - currentPosition.point.x + curveAimOffset.x,
		 target.point.y - currentPosition.point.y + curveAimOffset.y);
		maxTurnPower = 1;
	}
	else {
		pointAngle = target.angle;
		maxTurnPower = 0.5;
	}

	double angleDifference = pointAngle/M_PI*180 - Robot::gyro->GetAngle();
	double turnPower = kTurning * (angleDifference);

	double currentCentripetal = fabs(Robot::gyro->GetRate()/180*M_PI * Robot::drivetrain.GetRate());
	if (currentCentripetal > maxCentripetal) {
		turnPower /= pow(2, (currentCentripetal - maxCentripetal) * kTurnReduction);
	}
	if (fabs(turnPower) > maxTurnPower) turnPower = copysign(maxTurnPower, turnPower);

	double forwardPower;
	if (target.slowDown) {
		forwardPower = std::min(maxPower, minForwardPower + kForwardPower *
			// distance to target:
			sqrt(pow(target.point.x - currentPosition.point.x, 2)
		 + pow(target.point.y - currentPosition.point.y, 2)));
	}
	else forwardPower = maxPower;

	Robot::drivetrain.DrivePolar(forwardPower, -turnPower);
}

void AutoDrive::updatePosition() {
	RobotPosition newPos;

	newPos.encoderDistance = Robot::drivetrain.GetDistance();
	newPos.angle = Robot::gyro->GetAngle();
	double distance = newPos.encoderDistance - currentPosition.encoderDistance;
	
	newPos.point = { currentPosition.point.x + distance * sin(newPos.angle/180*M_PI),
	                 currentPosition.point.y + distance * cos(newPos.angle/180*M_PI) };

	currentPosition = newPos;
}