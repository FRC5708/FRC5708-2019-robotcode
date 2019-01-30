#pragma once

#include <frc/commands/Subsystem.h>
#include <chrono>

class AutoDrive : public frc::Subsystem {

public: 
	struct Point { double x, y; };

	// inches and degrees. Measured from center of hab, against wall, facing forward.
	// left is negative, right positive. Forward is positive, behind the wall would be negative.
	// clockwise positive, counterclockwise negative.
	struct RobotPosition {
		Point point;
		double angle; 
		double encoderDistance; // revolutions
	};
	RobotPosition currentPosition;

	struct Target {
		Point point;
		bool isAngled, slowDown;
		double angle;
	};
	Target target;

	AutoDrive();
	void Periodic() override;

private:
	std::chrono::steady_clock clock;

	void updatePosition();
};