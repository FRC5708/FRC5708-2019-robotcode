#pragma once

#include <frc/commands/Subsystem.h>
#include "subsystems/AutoDrive.h"
#include <string>
#include <sstream>
#include <vector>
#include "Angle.h"

class VisionReceiver : public frc::Subsystem {
 private:
	void setupSocket();
	void pollForData();

	int sockfd;

	struct TargetData {
		double distance;
		Radian tapeAngle, robotAngle;
	};

 public:
	VisionReceiver();
	

	// Subsystem Periodic() functions are always called before command Execute() functions
	void Periodic() override;

	struct TargetLoc {
		AutoDrive::Point loc;
		// angle (radians) from facing directly away from Hab. clockwise=positive.
		Radian angle;
	};
	std::vector<TargetLoc> targetLocs;
	std::vector<TargetLoc>& grabData() {
		newData = false;
		return targetLocs;
	}

	int processingTime; // milliseconds
	bool newData = false;
};
