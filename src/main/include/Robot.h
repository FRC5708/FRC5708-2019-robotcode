/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/TimedRobot.h>
#include <frc/commands/Command.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Joystick.h>
#include <frc/interfaces/Gyro.h>
#include <frc/DigitalInput.h>

#include "OI.h"
#include "subsystems/Drivetrain.h"
#include "subsystems/AutoDrive.h"
#include "subsystems/VisionReceiver.h"

#include "subsystems/ShiftieLiftie.h"
#include "subsystems/McShootieTube.h"
#include "subsystems/HatchManipulator.h"

class Robot : public frc::TimedRobot {
 public:
 
	static frc::Joystick *driveJoystick, *liftJoystick;
	static OI* m_oi;
	static Drivetrain drivetrain;
    static AutoDrive autoDrive;
    static VisionReceiver visionReceiver;
	static McShootieTube manipulator;
	static ShiftieLiftie lift;
	static frc::Gyro* gyro;
	static HatchManipulator hatch;

	static Robot* instance;
	static frc::DigitalInput* ProgrammaticUpperLimitSwitch;

	void RobotInit() override;
	void RobotPeriodic() override;
	void DisabledInit() override;
	void DisabledPeriodic() override;
	void AutonomousInit() override;
	void AutonomousPeriodic() override;
	void TeleopInit() override;
	void TeleopPeriodic() override;
	void TestPeriodic() override;

 private:
	// Have it null by default so that if testing teleop it
	// doesn't have undefined behavior and potentially crash.
	frc::Command* autoCommand = nullptr;
	//ExampleCommand m_defaultAuto;
	//MyAutoCommand m_myAuto;
	//frc::SendableChooser<frc::Command*> m_chooser;
	frc::SendableChooser<char> locationSelect;
	frc::SendableChooser<char> targetSideSelect;
	frc::SendableChooser<int> targetSelect;
	frc::SendableChooser<bool> itemSelect;

	frc::Command* driveCommand;
};
