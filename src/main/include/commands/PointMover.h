#pragma once

#include <frc/commands/Command.h>
#include <vector>

#include "subsystems/AutoDrive.h"

class PointMover : public frc::Command {
public:
	void Execute() override;
	void Initialize() override;
	void End() override;
	bool IsFinished() override { return done; }

	PointMover(std::vector<AutoDrive::Point> targets);

private:
	std::vector<AutoDrive::Point> targets;
	int currentTargetIdx;

	AutoDrive::Point prevTarget;

	void updateTarget();

	bool done;
};