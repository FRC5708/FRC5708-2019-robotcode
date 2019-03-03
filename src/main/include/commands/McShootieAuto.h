/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#ifndef SRC_COMMANDS_BALLSHOOT_H_
#define SRC_COMMANDS_BALLSHOOT_H_
#include <frc/commands/Command.h>
#include "Robot.h"
#include "subsystems/McShootieTube.h"
#include <chrono>

class BallShoot : public frc::Command {
public:
	BallShoot(){};
	
	void Initialize() override {
		startTime = std::chrono::steady_clock::now();
	}
	void Execute() override {
		Robot::manipulator.Shoot();
	}
	bool IsFinished() override {
		if (std::chrono::steady_clock::now() > startTime + std::chrono::milliseconds(2000)) {
			Robot::manipulator.Stop();
			return true;
		}
		return false;
	}
		private:
	std::chrono::steady_clock::time_point startTime;
};     
#endif 