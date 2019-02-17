#include "commands/VisionDrive.h"
#include "Robot.h"
#include <iostream>

VisionDrive::VisionDrive(bool retry) : retry(retry) {
	Requires(&Robot::autoDrive);
}

// Called just before this Command runs the first time
void VisionDrive::Initialize() {
	std::cout << "VisionDrive::Initialize()" << std::endl;
	Robot::autoDrive.commandUsing = this;
	gotFirstData = false;
	done = false;

	startingPoint = Robot::autoDrive.getCurrentPos().loc;

	if (Robot::visionReceiver.targetLocs.size() > 0 && Robot::visionReceiver.dataAge < 25) processVisionData();
	else if (!retry) done = true;
}

// Called repeatedly when this Command is scheduled to run
void VisionDrive::Execute() {
	if (done) Initialize();
	
	if (Robot::visionReceiver.targetLocs.size() > 0 && Robot::visionReceiver.newData) {
		processVisionData();
	}

	if (gotFirstData) {
		// distance, in inches, away from the vision targets, needed to turn without hitting anything
		constexpr double approachDist = 30;
		AutoDrive::Point approachPoint = { 
			currentTarget.loc.x - (approachDist + ROBOT_LENGTH / 2)*sin(currentTarget.angle),
			currentTarget.loc.y - (approachDist + ROBOT_LENGTH / 2)*cos(currentTarget.angle)
		};

		Robot::autoDrive.target.loc = approachPoint;
		
		Robot::autoDrive.target.isAngled = true;
		Robot::autoDrive.target.angle = currentTarget.angle;
		Robot::autoDrive.target.slowDown = false;

		if (// if we are closer than 2 inches to the target
			sqrt(pow(Robot::autoDrive.getCurrentPos().loc.x - currentTarget.loc.x, 2) + 
			pow(Robot::autoDrive.getCurrentPos().loc.y - currentTarget.loc.y, 2)) < 2) {
				
			std::cout << "at target!" << std::endl;
			done = true;
		}

		if (Robot::autoDrive.passedTarget(startingPoint)) {
			Robot::autoDrive.target.loc = {
				currentTarget.loc.x - (ROBOT_LENGTH / 2)*sin(currentTarget.angle),
				currentTarget.loc.y - (ROBOT_LENGTH / 2)*cos(currentTarget.angle)
			};
			Robot::autoDrive.target.slowDown = true;
			Robot::autoDrive.target.isAngled = false;
			Robot::autoDrive.output << "driving final foot; ";

			if (Robot::autoDrive.passedTarget(approachPoint)) {
				std::cout << "passed target!" << std::endl;
				done = true;
			}
		}
	}

	Robot::autoDrive.updatePower();
}

void VisionDrive::processVisionData() {
		std::cout << "Proccessing vision data..." << std::endl;

	double locationTolerance = 8; // inches
	if (!gotFirstData) locationTolerance = INFINITY;

	double bestDistance = locationTolerance;
	VisionReceiver::TargetLoc bestTarget;

	for (auto i : Robot::visionReceiver.grabData()) {

		double distance = sqrt(pow(i.loc.x - currentTarget.loc.x, 2) + 
		pow(i.loc.y - currentTarget.loc.y, 2));
		
		if (distance < bestDistance) {
			bestDistance = distance;
			bestTarget = i;
		}
	}
	if (bestDistance < locationTolerance) {
		currentTarget = bestTarget;
		Robot::autoDrive.output << "new target!: <" << 
		bestTarget.loc.x << ", " << bestTarget.loc.y <<"> theta=" << bestTarget.angle << std::endl;
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
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
// default implementation calls End()
//void VisionDrive::Interrupted() {}
