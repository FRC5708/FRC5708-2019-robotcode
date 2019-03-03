#include "commands/Autonomous.h"

#include "Robot.h"
#include <vector>

#include "subsystems/AutoDrive.h"

#include "commands/McShootieAuto.h"
#include "commands/VisionDrive.h"

class PointMover : public frc::Command {
public:
	void Execute() override;
	void Initialize() override;
	void End() override;
	bool IsFinished() override { return done; }

	PointMover(std::vector<AutoDrive::Point> targets) : targets(targets) {};

private:
	std::vector<AutoDrive::Point> targets;
	int currentTargetIdx;

	AutoDrive::Point prevTarget;

	void updateTarget();

	bool done;
};

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


Autonomous::Autonomous() {

	AddSequential(new PointMover(std::vector<AutoDrive::Point>({
		// add points here
	})));

	// TODO: something something Cancel() won't work
	AddSequential(new VisionDrive(true));
	AddSequential(new LiftMove(ShiftieLiftie::Setpoint::LowGoal));
	AddSequential(new BallShoot());
}