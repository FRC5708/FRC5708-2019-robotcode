#include "commands/PointMover.h"

#include "Robot.h"

void PointMover::Initialize() {

	Robot::autoDrive.target.slowDown = false;
	currentTargetIdx = 0;
	updateTarget();

	Robot::autoDrive.commandUsing = this;
}

void PointMover::Execute() {

	if (Robot::autoDrive.passedTarget(prevTarget)) {

		currentTargetIdx++;
		if (currentTargetIdx < targets.size()) {
			updateTarget();            
		}
		else done = true;
	}
	Robot::autoDrive.updatePower();
}

void PointMover::updateTarget() {
	prevTarget = Robot::autoDrive.getCurrentPos().loc;
	Robot::autoDrive.target.loc = targets[currentTargetIdx];

	if (currentTargetIdx == targets.size() - 1) {
		Robot::autoDrive.target.slowDown = true;
	}
}

void PointMover::End() {
	Robot::autoDrive.commandUsing = nullptr;
}