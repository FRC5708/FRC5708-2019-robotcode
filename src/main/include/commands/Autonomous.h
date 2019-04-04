#pragma once

#include <frc/commands/CommandGroup.h>
#include "subsystems/AutoDrive.h"

class DriveAndVision : public frc::CommandGroup {
public:
	DriveAndVision(std::vector<AutoDrive::Point> points, bool doCargo);
};

class PointMover : public frc::Command {
public:
	void Execute() override;
	void Initialize() override;
	void End() override;
	bool IsFinished() override { return done; }

	PointMover(std::vector<AutoDrive::Point> targets) : targets(targets) {};

protected:
	std::vector<AutoDrive::Point> targets;
private:
	int currentTargetIdx;

	AutoDrive::Point prevTarget;

	void updateTarget();

	bool done = false;
};


class RecoverFromVision : public PointMover {
public:
	RecoverFromVision(AutoDrive::Point pos, float backUpDist, bool front, bool pStation);
	void Initialize() override;
	void Execute() override;
	AutoDrive::Point getPoint() { return targets[0]; }

	private:
	AutoDrive::Point resetPos;
	
};
