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

	bool done = false;
};

void PointMover::Initialize() {

	Robot::autoDrive.target.slowDown = false;
	currentTargetIdx = 0;
	updateTarget();

	Robot::autoDrive.commandUsing = this;
}
void PointMover::Execute() {

	if (Robot::autoDrive.passedTarget(prevTarget) || Robot::autoDrive.atTarget(targets[currentTargetIdx])) {

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

class TimedHatch : public frc::Command {
public:
	TimedHatch(){};
	
	void Initialize() override {
		startTime = std::chrono::steady_clock::now();
	}
	void Execute() override {
		Robot::hatch.hatchMotor->Set(1);
	}
	bool IsFinished() override {
		if (std::chrono::steady_clock::now() > startTime + std::chrono::milliseconds(10000)) {
			Robot::hatch.hatchMotor->Set(0);
			return true;
		}
		return false;
	}
	void End() override {
		Robot::hatch.hatchMotor->Set(0);
	}
private:
	std::chrono::steady_clock::time_point startTime;
}; 
class WaitFor : public frc::Command {
	frc::Command* target;
public:
	WaitFor(frc::Command* target) : target(target) {};
	bool IsFinished() override {
		return !target->IsRunning();
	}
};


Autonomous::Autonomous(std::vector<AutoDrive::Point> points, bool doCargo) {
	TimedHatch* timedHatch = new TimedHatch();
	if (!doCargo) AddParallel(timedHatch);


	AddSequential(new PointMover(points));
	
	if (!doCargo) AddSequential(new WaitFor(timedHatch));
	AddSequential(new VisionDrive(true, !doCargo));

	if (doCargo) {
		AddSequential(new LiftMove(ShiftieLiftie::Setpoint::MidGoalCargo));
		AddSequential(new BallShoot());
	}
}