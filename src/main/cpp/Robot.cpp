/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include <iostream>

#include <frc/commands/Scheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/AnalogGyro.h>
#include <frc/ADXRS450_Gyro.h>
#include "commands/DriveWithJoystick.h"
#include <sys/stat.h>

bool environment_check();
bool IS_PROD = environment_check();
frc::Joystick *Robot::driveJoystick, *Robot::liftJoystick;
OI* Robot::m_oi;
Drivetrain Robot::drivetrain;
AutoDrive Robot::autoDrive;
VisionReceiver Robot::visionReceiver;
ShiftieLiftie Robot::lift;
McShootieTube Robot::manipulator;
frc::Gyro* Robot::gyro;

Robot* Robot::instance;
bool environment_check(){
struct stat buffer;  
IS_PROD = !(stat ("/home/lvuser/platform_test", &buffer) == 0);
std::cout << "Platform detected as " << (IS_PROD ? "PROD" : "TEST" )<< "..." << std::endl;
}
void Robot::RobotInit() {
	instance = this;
	//m_chooser.SetDefaultOption("Default Auto", &m_defaultAuto);
	//m_chooser.AddOption("My Auto", &m_myAuto);
	//frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

	Robot::driveJoystick = new frc::Joystick(0);
	Robot::liftJoystick = new frc::Joystick(1);
	Robot::gyro = new frc::ADXRS450_Gyro();
	Robot::m_oi = new OI();

	driveCommand = new DriveWithJoystick();
	
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {
	frc::Scheduler::GetInstance()->RemoveAll();
}

void Robot::DisabledPeriodic() { frc::Scheduler::GetInstance()->Run(); }

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString code to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional commands to the
 * chooser code above (like the commented example) or additional comparisons to
 * the if-else structure below with additional strings & commands.
 */
void Robot::AutonomousInit() {
	// std::string autoSelected = frc::SmartDashboard::GetString(
	//     "Auto Selector", "Default");
	// if (autoSelected == "My Auto") {
	//   m_autonomousCommand = &m_myAuto;
	// } else {
	//   m_autonomousCommand = &m_defaultAuto;
	// }

	m_autonomousCommand = m_chooser.GetSelected();

	if (m_autonomousCommand != nullptr) {
		m_autonomousCommand->Start();
	}
}

void Robot::AutonomousPeriodic() { frc::Scheduler::GetInstance()->Run(); }

void Robot::TeleopInit() {
	// This makes sure that the autonomous stops running when
	// teleop starts running. If you want the autonomous to
	// continue until interrupted by another command, remove
	// this line or comment it out.
	if (m_autonomousCommand != nullptr) {
		m_autonomousCommand->Cancel();
		m_autonomousCommand = nullptr;
	}
	driveCommand->Start();
}

void Robot::TeleopPeriodic() { frc::Scheduler::GetInstance()->Run(); }

void Robot::TestPeriodic() {
	std::cout << "gyro angle: " << Robot::gyro->GetAngle() << std::endl;
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
