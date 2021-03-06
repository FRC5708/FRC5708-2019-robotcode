/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */

// For example to map the left and right motors, you could define the
// following variables to use with your drivetrain subsystem.
// constexpr int kLeftMotor = 1;
// constexpr int kRightMotor = 2;

// If you are using multiple modules, make sure to define both the port
// number and the module. For example you with a rangefinder:
// constexpr int kRangeFinderPort = 1;
// constexpr int kRangeFinderModule = 1;


constexpr int LeftEncoderChannel[2] = {0,1};
constexpr int RightEncoderChannel[2] = {2,3};
constexpr int LiftEncoderChannel[2] = {4,5};
constexpr int HatchCounterChannel = 6;
constexpr int programmaticUpperLimitSwitchChannel = 9;

constexpr int FLMotorChannel = 0;
constexpr int BLMotorChannel = 1;
constexpr int FRMotorChannel = 2;
constexpr int BRMotorChannel = 3;
constexpr int liftMotorChannel = 4;
constexpr int ballManipulatorMotorLeft = 5, ballManipulatorMotorRight = 6;
constexpr int hatchManipulatorChannel=7;

constexpr double WheelCircumference = 6 * M_PI;

constexpr double ROBOT_WIDTH = 27.5; // inches
constexpr double ROBOT_LENGTH = 33;
extern bool IS_PROD;