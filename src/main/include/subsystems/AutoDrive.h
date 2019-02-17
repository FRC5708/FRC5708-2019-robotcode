#pragma once

#include <frc/commands/Subsystem.h>
#include <chrono>
#include "Angle.h"
#include <iostream>
#include <fstream>
#include <pathfinder.h>
#include <RobotMap.h>

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

	// Call this from the command that is using AutoDrive.
	// Wpilib's scheduler calls subsystems before commands, 
	// so we can't just use Subsystem::Periodic().
	void updatePower();

	AutoDrive();

	frc::Command* commandUsing = nullptr;
	int max_speed = 120;
	double testing_speed = max_speed;// w/ 4;
	EncoderConfig config_l= {0/*encoder_position*/, 360/*ticks/rev*/, WheelCircumference/*wheel_circ*/, 1.0/*kp*/, 0.0/*ki*/, 0.0/*kd*/, 1.0 / testing_speed/*kv*/, 0.0/*ka*/}; 
	EncoderConfig config_r= {0/*encoder_position*/, 360/*ticks/rev*/, WheelCircumference/*wheel_circ*/, 1.0/*kp*/, 0.0/*ki*/, 0.0/*kd*/, 1.0 / testing_speed/*kv*/, 0.0/*ka*/}; 
	EncoderFollower follower_l;
	EncoderFollower follower_r;
	Segment* leftTrajectory;
	Segment* rightTrajectory;
	void pathfinderGeneratePath();
	void pathfinderFollowPath();
	void pathfinderDo();
	bool pathExists=false;

private:
	void updatePosition();
	
	std::chrono::steady_clock clock;

	static constexpr int posCount = 500;
	RobotPosition positions[posCount]; // 10 seconds
	int currentPosIndex = 0;
};