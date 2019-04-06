/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include "commands/Autonomous.h"
#include <iostream>
#include <frc/commands/Scheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/AnalogGyro.h>
#include <frc/ADXRS450_Gyro.h>
#include "commands/DriveWithJoystick.h"
#include <sys/stat.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DigitalInput.h>

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
HatchManipulator Robot::hatch;
frc::DigitalInput* Robot::ProgrammaticUpperLimitSwitch = new frc::DigitalInput(programmaticUpperLimitSwitchChannel);

Robot* Robot::instance;
bool environment_check(){
struct stat buffer;  
bool IS_PROD_face = !(stat ("/home/lvuser/platform_test", &buffer) == 0);
std::cout << "Platform detected as " << (IS_PROD_face ? "PROD" : "TEST" )<< "..." << std::endl;
return IS_PROD_face; 
}
void setupTargetSelect(frc::SendableChooser<char>& targetSideSelect, 
frc::SendableChooser<int>& targetSelect, 
std::string prefix = "") {

	targetSideSelect.SetDefaultOption("Left", 'L');
	targetSideSelect.AddOption("Right", 'R');
	frc::SmartDashboard::PutData(prefix + "Target Side", &targetSideSelect);

	targetSelect.SetDefaultOption("Ship Front", 0);
	targetSelect.AddOption("Ship Side 1", 1);
	targetSelect.AddOption("Ship Side 2", 2);
	targetSelect.AddOption("Ship Side 3", 3);

	frc::SmartDashboard::PutData(prefix + "Target", &targetSelect);
}


void Robot::RobotInit() {
	instance = this;
	//m_chooser.SetDefaultOption("Default Auto", &m_defaultAuto);
	//m_chooser.AddOption("My Auto", &m_myAuto);
	//frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

	Robot::driveJoystick = new frc::Joystick(0);
	Robot::liftJoystick = new frc::Joystick(0);//FIXME
	Robot::gyro = new frc::ADXRS450_Gyro();
	Robot::m_oi = new OI();

	driveCommand = new DriveWithJoystick();

	locationSelect.SetDefaultOption("Center", 'C');
	locationSelect.AddOption("Left", 'L');
	locationSelect.AddOption("Right", 'R');
	frc::SmartDashboard::PutData("Starting Pos", &locationSelect);

	setupTargetSelect(targetSideSelect, targetSelect);

	itemSelect.SetDefaultOption("Cargo", true);
	itemSelect.AddOption("Hatch", false);
	frc::SmartDashboard::PutData("Game Piece", &itemSelect);

	playerStationSelect.SetDefaultOption("Don't", PSPos::Dont);
	playerStationSelect.AddOption("Left", PSPos::Left);
	playerStationSelect.AddOption("Right", PSPos::Right);
	frc::SmartDashboard::PutData("Player Station", &playerStationSelect);

	setupTargetSelect(secondTargetSideSelect, secondTargetSelect, "Second ");
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
	frc::SmartDashboard::PutNumberArray("encoders", { 
		drivetrain.leftEncoder->GetDistance(),
		drivetrain.rightEncoder->GetDistance()
	});
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {
	frc::Scheduler::GetInstance()->RemoveAll();
	visionReceiver.sendControlHeartbeat();
}

void Robot::DisabledPeriodic() { 
	frc::Scheduler::GetInstance()->Run();

	frc::SmartDashboard::PutString("Auto", "pos: "+
	std::to_string(locationSelect.GetSelected())
	+ ". Target: "+std::to_string(targetSelect.GetSelected())
	+ ", side: "+std::to_string(targetSideSelect.GetSelected()));
}

constexpr double SHIP_FRONT_Y = 220;
// returns goal location
AutoDrive::Point toCargoShip(float sideSign, int target, std::vector<AutoDrive::Point>& points) {
	AutoDrive::Point toReturn;
	
	if (target == 0) {
		points.push_back({ 18*sideSign, SHIP_FRONT_Y - 35 - ROBOT_LENGTH/2 });
		points.push_back({ points.back().x, SHIP_FRONT_Y - 25 - ROBOT_LENGTH/2 });
		toReturn = { points.back().x, SHIP_FRONT_Y - ROBOT_LENGTH/2 };
	}
	else {
		points.push_back({ (23+36)*sideSign, SHIP_FRONT_Y - ROBOT_LENGTH/2 - 6 });
		points.push_back({ points.back().x, SHIP_FRONT_Y + 25 + (target - 1) * 21 });
		points.push_back({ points.back().x - 12*sideSign, points.back().y });
		toReturn = { ((4*12 + 7.75)/2 + ROBOT_LENGTH/2) * sideSign, points.back().y };
	}

	std::cout << "auto points: ";
	for (auto i : points) {
		std::cout << "(" << i.x << "," << i.y << ")  "; 
	}
	std::cout << std::endl;

	return toReturn;
}


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
	std::cout << "Robot::AutonomousInit" << std::endl;
	visionReceiver.sendControlHeartbeat();
	// std::string autoSelected = frc::SmartDashboard::GetString(
	//     "Auto Selector", "Default");
	// if (autoSelected == "My Auto") {
	//   m_autonomousCommand = &m_myAuto;
	// } else {
	//   m_autonomousCommand = &m_defaultAuto;
	// }

	/*m_autonomousCommand = m_chooser.GetSelected();

	if (m_autonomousCommand != nullptr) {
		m_autonomousCommand->Start();
	}*/

	if (autoCommand != nullptr) delete autoCommand;
	autoCommand = new frc::CommandGroup();

	drivetrain.ResetDistance();
	gyro->Reset();

	if (!driveCommand->IsRunning()) driveCommand->Start();

	double xMag = 2*12 + 3*12 + 4 - ROBOT_WIDTH/2;
	double yStart = 4*12 + ROBOT_LENGTH/2;

	AutoDrive::Point start;
	switch (locationSelect.GetSelected()) {
		case 'R': start = { xMag, yStart }; break;
		case 'L': start = { -xMag, yStart }; break;
		default: 
		std::cout << "Bad locationSelect" << std::endl;
		case 'C': start = { 0, yStart }; break;
	}

	autoDrive.resetPosition({ start, 0, drivetrain.GetDistance() });

	float sideSign = ((targetSideSelect.GetSelected() == 'L') ? -1 : 1);

	std::vector<AutoDrive::Point> points;
	points.push_back({ autoDrive.getCurrentPos().loc.x, autoDrive.getCurrentPos().loc.y + 4*12 + 6 });

	int target1 = targetSelect.GetSelected();
	auto endingPoint = toCargoShip(sideSign, target1, points);
	autoCommand->AddSequential(new DriveAndVision(points, itemSelect.GetSelected()));
	
	if (playerStationSelect.GetSelected() != PSPos::Dont) {

		RecoverFromVision* resetCommand = new RecoverFromVision(endingPoint, 18, (target1 == 0), false);
		autoCommand->AddSequential(resetCommand);

		// player station
		std::vector<AutoDrive::Point> points2;
		float sideSign2 = ((playerStationSelect.GetSelected() == PSPos::Left) ? -1 : 1);

		points2.push_back({ resetCommand->getPoint().x, SHIP_FRONT_Y - 36 });
		points2.push_back({ (5*12 + 4 + 69.56)*sideSign2, points.back().y });
		points2.push_back({ points.back().x, 0 });
		
		autoCommand->AddSequential(new DriveAndVision(points2, false));
		autoCommand->AddSequential(new RecoverFromVision(
			{ points2.back().x, ROBOT_LENGTH/2 }, 10*12, false, true));

		// second hatch delivery
		std::vector<AutoDrive::Point> points3;

		toCargoShip(((secondTargetSideSelect.GetSelected() == 'L') ? -1 : 1), secondTargetSelect.GetSelected(), points3);
		autoCommand->AddSequential(new DriveAndVision(points, false));
	}

	autoCommand->Start();
}

void Robot::AutonomousPeriodic() { frc::Scheduler::GetInstance()->Run(); }

void Robot::TeleopInit() {
	visionReceiver.sendControlHeartbeat();
	if (!driveCommand->IsRunning()) driveCommand->Start();
}

void Robot::TeleopPeriodic() { frc::Scheduler::GetInstance()->Run(); }

void Robot::TestPeriodic() {
	std::cout << "gyro angle: " << Robot::gyro->GetAngle() << std::endl;
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
