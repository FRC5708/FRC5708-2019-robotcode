/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <frc/commands/Scheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Spark.h>
#include <frc/Joystick.h>

#include <unistd.h>
#include <chrono>
#include <thread>

ExampleSubsystem Robot::m_subsystem;
OI Robot::m_oi;

void Robot::RobotInit() {
	m_chooser.SetDefaultOption("Default Auto", &m_defaultAuto);
	m_chooser.AddOption("My Auto", &m_myAuto);
	frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
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
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() { frc::Scheduler::GetInstance()->Run(); }

struct Note { double time, freq; };
static const Note notes[] = {
	{ 1.5, 40 },
	{ 1.5, 60 },
	{ 3, 80 }
};

class MotorMusic {
	public:
	volatile double frequency = 40;
	frc::Spark motor;
	std::chrono::steady_clock clock;

	MotorMusic(): motor(1) {}

	void play() {
		int currentNote;
		std::chrono::time_point<std::chrono::steady_clock> noteStart;

		while (true) {
			motor.Set(-0.2);
			usleep(1.0 / frequency * 1000000 / 2);
			motor.Set(0.2);
			usleep(1.0 / frequency * 1000000 / 2);
			/*if (std::chrono::duration<double>(noteStart - clock.now()).count() >= notes[currentNote].time) {
				noteStart = clock.now();
				++currentNote;
				currentNote = currentNote % (sizeof(notes) / (sizeof(double) * 2));
				frequency = notes[currentNote].freq;
			}*/
		}
	}
};


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

	static MotorMusic* music = new MotorMusic();

	std::thread([]() {
		music->play();
	}).detach();
	//music->play();
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
}

frc::Joystick joystick(0);
//frc::Spark motor1(0);
//frc::Spark motor2(1);

void Robot::TeleopPeriodic() { 
	frc::Scheduler::GetInstance()->Run();

	if (joystick.GetRawButton(1)) {
		//motor1.Set(1);
		//motor2.Set(1);
	}
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
