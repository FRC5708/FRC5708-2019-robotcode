#pragma once

#include <frc/commands/Subsystem.h>
#include "subsystems/AutoDrive.h"
#include <string>
#include <sstream>
#include <vector>
#include <sys/socket.h>
#include "Angle.h"

class VisionReceiver : public frc::Subsystem {
 private:
	void setupSocket();
	void pollForData();

	void sendControlMessage(const char* message);

	int sockfd;
	struct sockaddr_storage clientAddr;
	socklen_t clientAddrLen = sizeof(clientAddr);
	bool clientAddrExists = false;

	struct TargetData {
		double distance;

		// see 2019-vision-rpi/vision.hpp for descriptions
		Radian tapeAngle, robotAngle;
	};

	int ticks;

 public:
	VisionReceiver();
	
	void RobotEnabled() { sendControlMessage("ENABLE"); }
	void RobotDisabled() { sendControlMessage("DISABLE"); }

	// Subsystem Periodic() functions are always called before command Execute() functions
	void Periodic() override;

	struct TargetLoc {
		AutoDrive::Point loc;
		// angle (radians) from facing directly towards the Hab (y=0). clockwise=positive.
		Radian angle;
	};
	std::vector<TargetLoc> targetLocs;
	std::vector<TargetLoc>& grabData() {
		newData = false;
		return targetLocs;
	}

	int processingTime; // milliseconds
	int dataAge = 0; // ticks
	bool newData = false;
};
