#include "commands/VisionDrive.h"
#include "Robot.h"
#include <iostream>

VisionDrive::VisionDrive(bool retry, bool stayBack) : retry(retry), stayBack(stayBack) {
	Requires(&Robot::autoDrive);
}

// Called just before this Command runs the first time
// We also call it every time the command starts running
void VisionDrive::Initialize() {
	std::cout << "VisionDrive::Initialize()" << std::endl;
	Robot::autoDrive.commandUsing = this;
	gotFirstData = false;
	done = false;

	startingPoint = Robot::autoDrive.getCurrentPos().loc;

	if (Robot::visionReceiver.targetLocs.size() > 0 && Robot::visionReceiver.dataAge < 25) processVisionData();
	else if (!retry) done = true;

	Robot::visionReceiver.setIsActivelyDriving(true);
}

// Called repeatedly when this Command is scheduled to run
void VisionDrive::Execute() {
	if (done) Initialize();
	
	if (Robot::visionReceiver.targetLocs.size() > 0 && Robot::visionReceiver.newData) {
		processVisionData();
	}

	if (gotFirstData) {
		if (fabs(Robot::drivetrain.GetRate()) < 1) stallCount++;
		if (stallCount > 50) {
			done = true;
			return;
		}

		// distance, in inches, away from the vision targets, needed to turn without hitting anything
		constexpr double approachDist = 30;

		Robot::autoDrive.target.backwards = false;

		AutoDrive::Point approachPoint = { 
			currentTarget.loc.x - (approachDist + ROBOT_LENGTH / 2)*sin(currentTarget.angle),
			currentTarget.loc.y - (approachDist + ROBOT_LENGTH / 2)*cos(currentTarget.angle)
		};

		Robot::autoDrive.target.loc = approachPoint;
		
		Robot::autoDrive.target.isAngled = true;
		Robot::autoDrive.target.angle = currentTarget.angle;
		Robot::autoDrive.target.slowDown = false;
		Robot::autoDrive.maxPower = 0.4; Robot::autoDrive.maxTurnPower = 0.5;

		if (// if we are closer than 2 inches to the target
		Robot::autoDrive.atTarget(currentTarget.loc, 2)) {
				
			std::cout << "at target!" << std::endl;
			done = true;
		}

		if (/*Robot::autoDrive.passedTarget(startingPoint)*/true) {
			//double finalApproachDist = stayBack ? 2 : 0;
			double finalApproachDist = 0;//-5;

			Robot::autoDrive.target.loc = {
				currentTarget.loc.x - (finalApproachDist + ROBOT_LENGTH / 2)*sin(currentTarget.angle),
				currentTarget.loc.y - (finalApproachDist + ROBOT_LENGTH / 2)*cos(currentTarget.angle)
			};
			Robot::autoDrive.target.slowDown = true;
			Robot::autoDrive.target.isAngled = false;
			Robot::autoDrive.maxPower = 0.4; Robot::autoDrive.maxTurnPower = 0.5;
			Robot::autoDrive.output << "driving final foot; ";

			if (Robot::autoDrive.passedTarget(approachPoint)) {
				std::cout << "passed target!" << std::endl;
				done = true;
			}
		}
		Robot::autoDrive.updatePower();
	}
	else Robot::drivetrain.Drive(0, 0);
}

void VisionDrive::processVisionData() {
		std::cout << "Proccessing vision data..." << std::endl;

	double locationTolerance = 8; // inches
	if (!gotFirstData) locationTolerance = INFINITY;

	double bestDistance = locationTolerance;
	VisionReceiver::TargetLoc bestTarget;

	for (auto i : Robot::visionReceiver.grabData()) {

		double distance = AutoDrive::pointDist(i.loc, currentTarget.loc);
		
		if (distance < bestDistance) {
			bestDistance = distance;
			bestTarget = i;
		}
	}
	if (bestDistance < locationTolerance) {
		currentTarget = bestTarget;
		Robot::autoDrive.output << "new target!: <" << 
		bestTarget.loc.x << ", " << bestTarget.loc.y <<"> theta=" << bestTarget.angle << '\n';
	}	

	gotFirstData = true;
}

// Make this return true when this Command no longer needs to run execute()
bool VisionDrive::IsFinished() { 
	if (IsCanceled()) done = true;
	return done;
}

// Called once after isFinished returns true
void VisionDrive::End() {
	std::cout << "ending VisionDrive" << std::endl;
	Robot::autoDrive.commandUsing = nullptr;
	Robot::visionReceiver.setIsActivelyDriving(false);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
// default implementation calls End()
//void VisionDrive::Interrupted() {}
