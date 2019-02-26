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

	// position of the CENTER of the robot.
	// inches and degrees. Measured from center of hab, against wall, facing forward.
	// left is negative, right positive. Forward is positive, behind the wall would be negative.
	// clockwise positive, counterclockwise negative.
	struct RobotPosition {
		Point loc;
		Degree angle; 
		double encoderDistance; // revolutions
	};
	RobotPosition& getCurrentPos() { return positions[currentPosIndex]; }
	RobotPosition getPastPos(double milliseconds);
	void resetPosition() { for (auto i : positions) { i = { 0, 0, 0, 0 }; } }

	struct Target {
		Point loc;

		bool isAngled, // whether angle is set
		 slowDown; // whether to slow down near the target
		Degree angle; // angle to end up at
	};
	Target target;

	bool passedTarget(Point from);
	bool atTarget(Point compare, double dist = 2);

	// Call this from the command that is using AutoDrive.
	// Wpilib's scheduler calls subsystems before commands, 
	// so we can't just use Subsystem::Periodic().
	void updatePower();

	AutoDrive();

	frc::Command* commandUsing = nullptr;

private:
	void updatePosition();

	Point getCurveAimOffset(double radius);
	Point matchingPair(double x, double y1, double y2, AutoDrive::Point center, double comp);
	Point tangentPoint(AutoDrive::Point center, AutoDrive::Point currentPos, double radius);
	
	std::chrono::steady_clock clock;

	static constexpr int posCount = 500;
	RobotPosition positions[posCount]; // 10 seconds
	int currentPosIndex = 0;

	Degree beforePassAngle;
};