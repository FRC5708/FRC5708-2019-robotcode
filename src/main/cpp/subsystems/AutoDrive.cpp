#include "subsystems/AutoDrive.h"
#include "Robot.h"

#include <algorithm>
#include <iostream>
#include <fstream>

#include <array>

AutoDrive::AutoDrive() : frc::Subsystem("AutoDrive") {
	resetPosition();
	output.open("/home/lvuser/position_output.txt", std::ofstream::out | std::ofstream::trunc);
}

constexpr double maxCentripetal = 6*12, // in/sec^2
 topSpeed = 4*12, // estimated top speed of the robot at full power, inches/sec
 reachTopSpeed = 1, // estimated seconds to reach top speed from stop
 maxPower = 0.4, // maximum power the auton will run the motors at
 minForwardPower = 0.25; // below this the robot won't move


AutoDrive::Point AutoDrive::matchingPair(double x, double y1, double y2, AutoDrive::Point center, double comp) {

	if (fabs(pow(x - center.x, 2) + pow(y1 - center.y, 2) - comp) < 0.00000000001) return {x, y1};
	else if (fabs(pow(x - center.x, 2) + pow(y2 - center.y, 2) - comp) < 0.00000000001) return {x, y2};

	else {
		std::cout << "Error in matchingPair()! Time to throw around some NaN!" << std::endl;
		output << "matchingPair() error. x:" << x << " y1:"  << y1 << " y2:" << y2 << 
		" center:x:" << center.x << " y:" << center.y << " r^2:" << comp << 
		" dist1:" << pow(x - center.x, 2) + pow(y1 - center.y, 2) << 
		" dist2:" << pow(x - center.x, 2) + pow(y2 - center.y, 2) << std::endl;
		
		return { NAN, NAN };
	}
}
// center=center of circle that robot will drive on
AutoDrive::Point AutoDrive::tangentPoint(AutoDrive::Point center, AutoDrive::Point currentPos, 
double radius) {
	// if inside circle
	if (pow(currentPos.x - center.x, 2) + pow(currentPos.y - center.y, 2) < pow(radius, 2)) {
		return { NAN, NAN };
	}

	double centerDist2 = pow(currentPos.x - center.x, 2) + pow(currentPos.y - center.y, 2);
	double centerDist = sqrt(centerDist2);
	double aimPointDist = sqrt(centerDist2 - pow(radius, 2));

	double xl = pow(radius, 2) * (currentPos.x - center.x) / centerDist2;
	double xr = radius*(currentPos.y - center.y)*aimPointDist / centerDist2;
	double x1 = xl - xr + center.x, x2 = xl + xr + center.x;

	double yl = pow(radius, 2) * (currentPos.y - center.y) / centerDist2;
	double yr = radius*(currentPos.x - center.x)*aimPointDist / centerDist2;
	double y1 = yl - yr + center.y, y2 = yl + yr + center.y;

	// we get 2 solutions each for x and y. Only two pairs are correct, 
	// but I don't wanna do the math to figure out which one, so instead, 
	//I'm checking if the points are on the circle

	auto p1 = matchingPair(x1, y1, y2, center, pow(radius, 2));
	auto p2 = matchingPair(x2, y1, y2, center, pow(radius, 2));

	// the solution to use is the closest to the target, which is at (0,0)
	return ((pow(p1.x, 2) + pow(p1.y, 2) < pow(p2.x, 2) + pow(p2.y, 2)) ? p1 : p2);
}

AutoDrive::Point AutoDrive::getCurveAimOffset(double radius) {
	
	Point p1 = tangentPoint({
		radius*cos(target.angle),
		-radius*sin(target.angle)
	}, getCurrentPos().loc, radius);

	Point p2 = tangentPoint({
		-radius*cos(target.angle),
		radius*sin(target.angle)
	}, getCurrentPos().loc, radius);

	if (isnan(p1.x) || isnan(p1.y) || isnan(p2.x) || isnan(p2.y)) return {0, 0};
	
	// I don't wanna do the math again. But the closest one is always the right one.
	return ((pow(getCurrentPos().loc.x - target.loc.x - p1.x, 2) + 
	pow(getCurrentPos().loc.y - target.loc.y - p1.y, 2) < 
	pow(getCurrentPos().loc.x - target.loc.x - p2.x, 2) + 
	pow(getCurrentPos().loc.y - target.loc.y - p2.y, 2))
	 ? p1 : p2);
}

// NOTE: assumes gyro is clockwise=positive
void AutoDrive::updatePower() {

	constexpr double kTurning = 0.1,
	 kTurnReduction = 0.05,
	 kForwardPower = 0.05;

	// Where to point in order to give enough space for the robot to turn
	Point curveAimOffset;
	if (false/*target.isAngled*/) {

		double targetDistance = sqrt(pow(target.loc.x - getCurrentPos().loc.x, 2) + 
		pow(target.loc.y - getCurrentPos().loc.y, 2));

		double expectedSpeed = targetDistance / reachTopSpeed;
		if (target.slowDown) expectedSpeed /= 2;
		expectedSpeed = std::max(Robot::drivetrain.GetRate(), std::min(expectedSpeed, topSpeed));

		// allow for a turn with 3/4 of the maxCentripetal
		double turningRadius = pow(expectedSpeed, 2) / (maxCentripetal * 0.75);

		//Radian targetAngle = target.angle; //Conversion between Radians and Degrees done implicitly. 
		//Radian lookAngle = atan2(target.loc.x - getCurrentPos().loc.x,
		// target.loc.y - getCurrentPos().loc.y);
		/*Radian sideRobotAngle = Degree(Robot::gyro->GetAngle() - 90.0);

		curveAimOffset.x = turningRadius * (sin(sideRobotAngle) - sin(targetAngle));
		curveAimOffset.y = turningRadius * (cos(targetAngle) - cos(sideRobotAngle));*/
		//curveAimOffset.x = turningRadius * (cos(lookAngle) - cos(targetAngle));
		//curveAimOffset.y = turningRadius * (sin(lookAngle) - sin(targetAngle));
 
		curveAimOffset = getCurveAimOffset(turningRadius);

/*
		if (curveAimOffset.x != 0 && (curveAimOffset.x > 0 ^ (target.loc.x - getCurrentPos().loc.x > 0)))
		std::cout << "aim offset weird x!" << std::endl;
		if (curveAimOffset.y != 0 && (curveAimOffset.y > 0 ^ (target.loc.y - getCurrentPos().loc.y > 0)))
			std::cout << "aim offset weird y!" << std::endl;*/
		
	}
	else curveAimOffset = { 0, 0 };

	Radian pointAngle;
	double maxTurnPower;
	// if we haven't passed the aim point
	//if (pow(target.loc.x - getCurrentPos().loc.x, 2) + pow(target.loc.y - getCurrentPos().loc.y, 2)
	//>= pow(curveAimOffset.x, 2) + pow(curveAimOffset.y, 2)) {
		// Point towards the curve aim point
		pointAngle = atan2(target.loc.x - getCurrentPos().loc.x + curveAimOffset.x,
		 target.loc.y - getCurrentPos().loc.y + curveAimOffset.y);
		maxTurnPower = 0.5;//1;
		//output << "aiming at aim point: offset: <" << curveAimOffset.x << ", " << curveAimOffset.y << ">" << std::endl;
	//}
	/*else {
		output << "aiming at target; ";
		pointAngle = atan2(target.loc.x - getCurrentPos().loc.x, target.loc.y - getCurrentPos().loc.y);

		// it will try to turn at this power, unless it is close to the target or already turning too fast
		maxTurnPower = 0.5;//std::max(0.5, 1 - Robot::drivetrain.GetRate() / topSpeed);
	}*/

	double turnPower = 0;
	Degree angleDifference = 0;

	if (!atTarget(target.loc, 4)) {
		// between -180 and 180
		angleDifference = remainder(Degree(pointAngle) - Robot::drivetrain.GetGyroAngle(), 360);

		beforePassAngle = Robot::drivetrain.GetGyroAngle();
	}
	else {
		// drive straight
		angleDifference = beforePassAngle - Robot::drivetrain.GetGyroAngle();
		output << "driving straight; ";
	}

	output << "angleDifference: " << angleDifference << "; ";
	turnPower = kTurning * angleDifference;

	double currentCentripetal = fabs(Radian(Robot::drivetrain.GetGyroRate()) * Robot::drivetrain.GetRate());
	if (currentCentripetal > maxCentripetal) {
		output << "turning less; ";
		turnPower /= pow(2, (currentCentripetal - maxCentripetal) * kTurnReduction);
	}
	if (fabs(turnPower) > maxTurnPower) turnPower = copysign(maxTurnPower, turnPower);

	double forwardPower = 0;
	if (fabs(angleDifference) < 30) {
		if (target.slowDown) {
			forwardPower = std::min(maxPower, minForwardPower + kForwardPower *
				// distance to target:
				sqrt(pow(target.loc.x - getCurrentPos().loc.x, 2)
			+ pow(target.loc.y - getCurrentPos().loc.y, 2)));
		}
		else forwardPower = maxPower;
	}
	
	output << "Driving polar with <" << forwardPower << "," << turnPower << "> ..." << std::endl;
	output << "Aiming at Target: <" << target.loc.x << "," << target.loc.y << "> theta=" << Degree(pointAngle) << std::endl;
	Robot::drivetrain.DrivePolar(forwardPower, turnPower);
}

bool AutoDrive::passedTarget(Point beginning) {
	// if we are farther from beginning than the target
	return pow(getCurrentPos().loc.x - beginning.x, 2) + pow(getCurrentPos().loc.y - beginning.y, 2)
	> pow(target.loc.x - beginning.x, 2) + pow(target.loc.y - beginning.y, 2);
}

bool AutoDrive::atTarget(Point compare, double dist) {
	return sqrt(pow(compare.x - getCurrentPos().loc.x, 2) + pow(compare.y - getCurrentPos().loc.y, 2)) < dist;
}

void AutoDrive::Periodic() {
	updatePosition();
}
void AutoDrive::updatePosition() {
	int newPosIdx = (currentPosIndex + 1) % posCount;
	RobotPosition& newPos = positions[newPosIdx];

	newPos.encoderDistance = Robot::drivetrain.GetDistance();
	newPos.angle = Robot::drivetrain.GetGyroAngle();
	double distance = newPos.encoderDistance - getCurrentPos().encoderDistance;
	
	newPos.loc = { getCurrentPos().loc.x + distance * sin(Radian(newPos.angle)),
	                 getCurrentPos().loc.y + distance * cos(Radian(newPos.angle)) };


	if (newPos.loc.x != getCurrentPos().loc.x || newPos.loc.y != getCurrentPos().loc.y) {
		output << "Current position: <" << getCurrentPos().loc.x 
		<< "," << getCurrentPos().loc.y << ">, theta=" << getCurrentPos().angle << std::endl;
	}

	currentPosIndex = newPosIdx;
}

AutoDrive::RobotPosition AutoDrive::getPastPos(double milliseconds) {
	int idxPast = round(milliseconds / (Robot::instance->GetPeriod() * 1000));
	//std::cout << "idxPast: " << idxPast << std::endl;
	if (idxPast >= posCount) {
		std::cerr << "requested time " << milliseconds << " too far in the past";
		return { 0, 0, 0, 0 };
		//abort();
	}
	return positions[(currentPosIndex - idxPast + posCount) % posCount];
}