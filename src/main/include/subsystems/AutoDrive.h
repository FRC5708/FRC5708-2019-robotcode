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
		Point loc;
		double angle; 
		double encoderDistance; // revolutions
	};
	RobotPosition currentPosition;

	struct Target {
		Point loc;
		bool isAngled, slowDown;
		double angle;
	};
	Target target;

	bool passedTarget(Point from);

	// Call this from the command that is using AutoDrive.
	// Wpilib's scheduler calls subsystems before commands, 
	// so we can't just use Subsystem::Periodic().
	void updatePower();

	AutoDrive();

private:
	std::chrono::steady_clock clock;

	void updatePosition();
};