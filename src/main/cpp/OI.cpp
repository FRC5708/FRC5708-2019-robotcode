/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "OI.h"
#include "Robot.h"

#include <frc/WPILib.h>
#include "commands/VisionDrive.h"

constexpr int VISION_BUTTON = 3;

OI::OI() {
	frc::JoystickButton* visionButton = new frc::JoystickButton(Robot::liftJoystick, VISION_BUTTON);
	VisionDrive* vd = new VisionDrive(false);
	visionButton->WhenPressed(vd);
	frc::JoystickButton* visionButton2 = new frc::JoystickButton(Robot::driveJoystick, VISION_BUTTON);
	visionButton2->WhenPressed(vd);
}
