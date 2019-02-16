#include "commands/VisionDrive.h"
#include "Robot.h"
#include <iostream>

VisionDrive::VisionDrive(bool retry) : retry(retry) {
	Requires(&Robot::autoDrive);
}

// Called just before this Command runs the first time
void VisionDrive::Initialize() {
	Robot::autoDrive.commandUsing = this;
	gotFirstData = false;

	startingPoint = Robot::autoDrive.getCurrentPos().loc;

	if (Robot::visionReceiver.targetLocs.size() > 0) processVisionData();
	else if (!retry) done = true;
}

// Called repeatedly when this Command is scheduled to run
void VisionDrive::Execute() {
	
	if (Robot::visionReceiver.targetLocs.size() > 0 && Robot::visionReceiver.newData) {
		//std::cout << "Proccessing vision data..." << std::endl;
		if (!gotFirstData) processVisionData();
	}

	if (gotFirstData) {
		// distance, in inches, away from the vision targets, needed to turn without hitting anything
		constexpr double approachDist = 12;

		/*Robot::autoDrive.target.loc = { 
			currentTarget.loc.x - approachDist*sin(currentTarget.angle),
			currentTarget.loc.y - approachDist*cos(currentTarget.angle)
		};*/
		
		Robot::autoDrive.target.isAngled = true;
		Robot::autoDrive.target.angle = currentTarget.angle;
		Robot::autoDrive.target.slowDown = false;
		/*
		if (Robot::autoDrive.passedTarget(startingPoint)) {
			Robot::autoDrive.target.loc = currentTarget.loc;
			Robot::autoDrive.target.slowDown = true;

			if (Robot::autoDrive.passedTarget(startingPoint)) {
				std::cout << "passed target!" << std::endl;
				done = true;
			}
		}*/
	}

	Robot::autoDrive.pathfinderGeneratePath();
}

void VisionDrive::processVisionData() {

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
