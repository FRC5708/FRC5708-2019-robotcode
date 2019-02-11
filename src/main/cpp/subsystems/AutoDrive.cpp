#include "subsystems/AutoDrive.h"
#include "Robot.h"

#include <algorithm>
#include <iostream>
#include <fstream>

AutoDrive::AutoDrive() : frc::Subsystem("AutoDrive"), 
currentPosition{ 0, 0, 0, 0 } {
	output.open("/home/lvuser/position_output.txt", std::ofstream::out | std::ofstream::trunc);
}

constexpr double maxCentripetal = 6*12, // in/sec^2
 topSpeed = 4*12, // estimated top speed of the robot at full power, inches/sec
 reachTopSpeed = 1, // estimated seconds to reach top speed from stop
 maxPower = 0.4, // maximum power the auton will run the motors at
 minForwardPower = 0.25; // below this the robot won't move


// NOTE: assumes gyro is clockwise=positive
void AutoDrive::updatePower() {

	constexpr double kTurning = 0.1,
	 kTurnReduction = 0.5,
	 kForwardPower = 0.05;

	// Where to point in order to give enough space for the robot to turn
	Point curveAimOffset;
	if (target.isAngled) {

		double targetDistance = sqrt(pow(target.loc.x - currentPosition.loc.x, 2) + 
		pow(target.loc.y - currentPosition.loc.y, 2));

		double expectedSpeed = targetDistance / reachTopSpeed;
		if (target.slowDown) expectedSpeed /= 2;
		expectedSpeed = std::max(Robot::drivetrain.GetRate(), std::min(expectedSpeed, topSpeed));

		// allow for a turn with 3/4 of the maxCentripetal
		double turningRadius = pow(expectedSpeed, 2) / (maxCentripetal * 0.75);

		Radian targetAngle = target.angle; //Conversion between Radians and Degrees done implicitly. 
		Radian sideRobotAngle = Degree(Robot::gyro->GetAngle() - 90.0);

		curveAimOffset.x = turningRadius * (sin(sideRobotAngle) - sin(targetAngle));
		curveAimOffset.y = turningRadius * (cos(targetAngle) - cos(sideRobotAngle));
	}
	else curveAimOffset = { 0, 0 };

	Radian pointAngle;
	double maxTurnPower;
	// if we haven't passed the aim point
	if (fabs(target.loc.x - currentPosition.loc.x) > fabs(curveAimOffset.x) &&
		fabs(target.loc.y - currentPosition.loc.y) > fabs(curveAimOffset.y)) {
		// Point towards the curve aim point
		pointAngle = atan2(target.loc.x - currentPosition.loc.x + curveAimOffset.x,
		 target.loc.y - currentPosition.loc.y + curveAimOffset.y);
		maxTurnPower = 0.5;//1;
		std::cout << "aiming at aim point; ";
	}
	else {
		std::cout << "aiming towards target; ";
		pointAngle = target.angle;

		// it will try to turn at this power, unless it is close to the target or already turning too fast
		maxTurnPower = 0.5;//std::max(0.5, 1 - Robot::drivetrain.GetRate() / topSpeed);
	}

	Degree angleDifference = Degree(pointAngle) - Degree(Robot::drivetrain.GetGyroAngle());
	double turnPower = kTurning * (angleDifference);

	double currentCentripetal = fabs(Robot::drivetrain.GetGyroRate()/180*M_PI * Robot::drivetrain.GetRate());
	if (currentCentripetal > maxCentripetal) {
		std::cout << "turning less; ";
		turnPower /= pow(2, (currentCentripetal - maxCentripetal) * kTurnReduction);
	}
	if (fabs(turnPower) > maxTurnPower) turnPower = copysign(maxTurnPower, turnPower);

	double forwardPower;
	if (target.slowDown) {
		forwardPower = std::min(maxPower, minForwardPower + kForwardPower *
			// distance to target:
			sqrt(pow(target.loc.x - currentPosition.loc.x, 2)
		 + pow(target.loc.y - currentPosition.loc.y, 2)));
	}
	else forwardPower = maxPower;
	std::cout << "Driving polar with <" << forwardPower << "," << turnPower << "> ..." << std::endl;
	output << "Aiming at Target: <" << target.loc.x << "," << target.loc.y << "> ..." << std::endl;
	Robot::drivetrain.DrivePolar(forwardPower, turnPower);
}

bool AutoDrive::passedTarget(Point beginning) {
	// if we are farther from beginning than the target
	return pow(currentPosition.loc.x - beginning.x, 2) + pow(currentPosition.loc.x - beginning.x, 2)
	> pow(target.loc.x - beginning.x, 2) + pow(target.loc.y - beginning.y, 2);
}
void AutoDrive::Periodic() {
	updatePosition();
}
void AutoDrive::updatePosition() {
	RobotPosition newPos;

	newPos.encoderDistance = Robot::drivetrain.GetDistance();
	newPos.angle = Robot::drivetrain.GetGyroAngle();
	double distance = newPos.encoderDistance - currentPosition.encoderDistance;
	
	newPos.loc = { currentPosition.loc.x + distance * sin(Radian(newPos.angle)),
	                 currentPosition.loc.y + distance * cos(Radian(newPos.angle)) };


	if (newPos.loc.x != currentPosition.loc.x || newPos.loc.y != currentPosition.loc.y) {
		output << "Current position: <" << currentPosition.loc.x 
		<< "," << currentPosition.loc.y << ">, theta=" << currentPosition.angle << std::endl;
	}
	currentPosition = newPos;
}