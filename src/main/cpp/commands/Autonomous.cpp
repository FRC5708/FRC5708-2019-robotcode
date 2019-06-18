#include "commands/Autonomous.h"

#include "Robot.h"
#include <vector>

#include "subsystems/AutoDrive.h"

#include "commands/McShootieAuto.h"
#include "commands/VisionDrive.h"
#include <frc/commands/InstantCommand.h>



void PointMover::Initialize() {

	Robot::autoDrive.target.slowDown = false;
	currentTargetIdx = 0;
	updateTarget();

	Robot::autoDrive.commandUsing = this;
	Robot::autoDrive.maxPower = 0.7; Robot::autoDrive.maxTurnPower = 1;
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

	//if (currentTargetIdx == targets.size() - 1) {
		//Robot::autoDrive.target.slowDown = true;
	//}
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
		Robot::hatch.hatchMotor->Set(-1);
	}
	bool IsFinished() override {
		if (std::chrono::steady_clock::now() > startTime + std::chrono::milliseconds(6000)) {
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

class CounterHatch : public frc::Command {
	void Initialize() override {
		Robot::hatch.Lower();
		// Race condition with hatch.Periodic() and IsFinished()
		// And don't want to look in WPI source code to find the order of these things
		Robot::hatch.isMoving = true;
	}
	bool IsFinished() override {
		return !Robot::hatch.isMoving;
	}
	void End() override {
		Robot::hatch.Stop();
	}
};

DriveAndVision::DriveAndVision(std::vector<AutoDrive::Point> points, bool doCargo, bool doVision) {
	frc::Command* hatchCommand = new TimedHatch();//new CounterHatch();
	if (!doCargo) AddParallel(hatchCommand);

	AddSequential(new PointMover(points));
	
	if (!doCargo) AddSequential(new WaitFor(hatchCommand));

	if (doVision) AddSequential(new VisionDrive(true, doCargo));

	if (doCargo) {
		AddSequential(new LiftMove(ShiftieLiftie::Setpoint::MidGoalCargo));
		AddSequential(new BallShoot());
	}
}

RecoverFromVision::RecoverFromVision(AutoDrive::Point pos, float backUpDist, bool front, bool pStation)
 : PointMover({}), resetPos(pos) {

	if (pStation)   targets = {{ pos.x, pos.y + backUpDist }};
	else if (front) targets = {{ pos.x, pos.y - backUpDist }};
	else targets = {{ pos.x - copysign(backUpDist, pos.x), pos.y }};
}
void RecoverFromVision::Initialize() {

	Robot::autoDrive.resetPosition({ resetPos, Robot::gyro->GetAngle(), Robot::drivetrain.GetDistance() });

	PointMover::Initialize();
}

void RecoverFromVision::Execute() {
	Robot::autoDrive.target.backwards = true;
	PointMover::Execute();
}