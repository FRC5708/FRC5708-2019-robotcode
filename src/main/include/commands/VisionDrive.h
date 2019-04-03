#pragma once

#include <frc/commands/Command.h>
#include "subsystems/AutoDrive.h"
#include "subsystems/VisionReceiver.h"

class VisionDrive : public frc::Command {
 public:
	VisionDrive(bool retry, bool stayBack = true);
	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	//void Interrupted() override;

private:
	// latency in milliseconds
	void processVisionData();

	AutoDrive::Point startingPoint;

	VisionReceiver::TargetLoc currentTarget;

	bool retry;
	bool gotFirstData = false;
	bool stayBack = true;

	bool done = false;
};
