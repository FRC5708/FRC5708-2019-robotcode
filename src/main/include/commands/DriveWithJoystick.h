/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Command.h>
#include <frc/Joystick.h>
#include <RobotMap.h>

#include "../Robot.h"

enum JoystickMode {SINGLE_JOY, XBOX};

class DriveWithJoystick : public frc::Command {
public:
	JoystickMode joyMode = XBOX;

	DriveWithJoystick();
	void Execute() override;
	bool IsFinished() override;
	void End() override;
};
