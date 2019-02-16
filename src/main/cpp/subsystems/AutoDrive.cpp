#include "subsystems/AutoDrive.h"
#include "Robot.h"

#include <algorithm>
#include <iostream>
#include <fstream>
#include <pathfinder.h>

AutoDrive::AutoDrive() : frc::Subsystem("AutoDrive") {
	resetPosition();
	output.open("/home/lvuser/position_output.txt", std::ofstream::out | std::ofstream::trunc);
}
/*
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

		double targetDistance = sqrt(pow(target.loc.x - getCurrentPos().loc.x, 2) + 
		pow(target.loc.y - getCurrentPos().loc.y, 2));
 
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
	if (fabs(target.loc.x - getCurrentPos().loc.x) > fabs(curveAimOffset.x) &&
		fabs(target.loc.y - getCurrentPos().loc.y) > fabs(curveAimOffset.y)) {
		// Point towards the curve aim point
		pointAngle = atan2(target.loc.x - getCurrentPos().loc.x + curveAimOffset.x,
		 target.loc.y - getCurrentPos().loc.y + curveAimOffset.y);
		maxTurnPower = 0.5;//1;
		output << "aiming at aim point; ";
	}
	else {
		output << "aiming towards target; ";
		pointAngle = atan2(target.loc.x - getCurrentPos().loc.x, target.loc.y - getCurrentPos().loc.y);

		// it will try to turn at this power, unless it is close to the target or already turning too fast
		maxTurnPower = 0.5;//std::max(0.5, 1 - Robot::drivetrain.GetRate() / topSpeed);
	}

	// between -180 and 180
	Degree angleDifference = remainder(Degree(pointAngle) - Robot::drivetrain.GetGyroAngle(), 360);
	output << "angleDifference: " << angleDifference << "; ";
	double turnPower = kTurning * angleDifference;

	double currentCentripetal = fabs(Radian(Robot::drivetrain.GetGyroRate()) * Robot::drivetrain.GetRate());
	if (currentCentripetal > maxCentripetal) {
		output << "turning less; ";
		turnPower /= pow(2, (currentCentripetal - maxCentripetal) * kTurnReduction);
	}
	if (fabs(turnPower) > maxTurnPower) turnPower = copysign(maxTurnPower, turnPower);

	double forwardPower = 0;
	if (fabs(angleDifference) < 60) {
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
	return pow(getCurrentPos().loc.x - beginning.x, 2) + pow(getCurrentPos().loc.x - beginning.x, 2)
	> pow(target.loc.x - beginning.x, 2) + pow(target.loc.y - beginning.y, 2);
}
*/
void AutoDrive::Periodic() {
	pathfinderDo();
	//updatePosition();
}

void AutoDrive::pathfinderGeneratePath(){
	pathExists=true;
	Waypoint points[2];
	points[0]=Waypoint{0,0,0};
	points[1]=Waypoint{target.loc.x - getCurrentPos().loc.x,target.loc.y - getCurrentPos().loc.y,target.angle};
	TrajectoryCandidate candidate;
	pathfinder_prepare(points, 2/*change?*/, FIT_HERMITE_CUBIC, PATHFINDER_SAMPLES_LOW, 0.001/*time_step*/, 15.0/*max_velocity*/, 10.0/*max_accel*/, 60.0/*max=jerk*/, &candidate);
	int length = candidate.length;
	Segment *trajectory = (Segment*) malloc(2*sizeof(Segment));
	pathfinder_generate(&candidate, trajectory);
	double wheelbase_width = 0.6;
	pathfinder_modify_tank(trajectory, length, leftTrajectory, rightTrajectory, wheelbase_width);
	follower_l.last_error = 0; follower_l.segment = 0; follower_l.finished = 0;
	follower_r.last_error = 0; follower_r.segment = 0; follower_r.finished = 0;
	config_l.initial_position=Robot::drivetrain.leftEncoder->GetDistance();
	config_r.initial_position=Robot::drivetrain.rightEncoder->GetDistance();
	std::cout << "New path generated ..." << std::endl;
}
void AutoDrive::pathfinderFollowPath(){
	std::cout << "Following path ... " << std::endl;
	double l = pathfinder_follow_encoder(config_l, &follower_l, leftTrajectory, 2, Robot::drivetrain.leftEncoder->GetDistance()/*enc_l_value*/);
	double r = pathfinder_follow_encoder(config_r, &follower_r, rightTrajectory,2, Robot::drivetrain.rightEncoder->GetDistance()/*enc_r_value*/);
	std::cout << "L: " << l << " R: " << r << std::endl;
	double gyro_heading =  Robot::drivetrain.GetGyroAngle() ; 
	double desired_heading = r2d(follower_l.heading);
	double angle_difference = desired_heading - gyro_heading;    // Make sure to bound this from -180 to 180, otherwise you will get super large values
	if(angle_difference>180.0) angle_difference-=180;
	if(angle_difference<-180.0) angle_difference+=180;
	double turn = 0.8 * (-1.0/80.0)/*right values?*/ * angle_difference;
	std::cout << "Attempting to drive <" << l + turn << "," << r - turn << ">" << std::endl;

	Robot::drivetrain.Drive(l + turn,r - turn);
}
void AutoDrive::pathfinderDo(){
  	//  pathfinderGeneratePath();
	if(pathExists) pathfinderFollowPath();
}

AutoDrive::RobotPosition AutoDrive::getPastPos(double milliseconds) {
	int idxPast = round(milliseconds / (Robot::instance->GetPeriod() * 1000));

	if (idxPast >= posCount) {
		std::cerr << "requested time " << milliseconds << " too far in the past";
		return {0, 0, 0, 0};
	}
	return positions[(currentPosIndex - idxPast + posCount) % posCount];
}