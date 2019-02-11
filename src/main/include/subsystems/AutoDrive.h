#pragma once

#include <frc/commands/Subsystem.h>
#include <chrono>
#include "Angle.h"
#include <iostream>
#include <fstream>


class AutoDrive : public frc::Subsystem {

public: 
	std::ofstream output;
	virtual void Periodic() override;

	struct Point { double x, y; };

	// inches and degrees. Measured from center of hab, against wall, facing forward.
	// left is negative, right positive. Forward is positive, behind the wall would be negative.
	// clockwise positive, counterclockwise negative.
	struct RobotPosition {
		Point loc;
		Degree angle; 
		double encoderDistance; // revolutions
	};
	RobotPosition currentPosition;

	struct Target {
		Point loc;

		bool isAngled, // whether angle is set
		 slowDown; // whether to slow down near the target
		Degree angle; // angle to end up at
	};
	Target target;

	bool passedTarget(Point from);

	// Call this from the command that is using AutoDrive.
	// Wpilib's scheduler calls subsystems before commands, 
	// so we can't just use Subsystem::Periodic().
	void updatePower();

	AutoDrive();

	frc::Command* commandUsing = nullptr;

private:
	std::chrono::steady_clock clock;

	void updatePosition();
};