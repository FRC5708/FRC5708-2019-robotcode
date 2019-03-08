/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/HatchManipulator.h"

HatchManipulator::HatchManipulator() : Subsystem("ExampleSubsystem") {}

void HatchManipulator::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
}/*
void HatchManipulator::Periodic(){
  
}*/
// Put methods for controlling this subsystem
// here. Call these from Commands.
void HatchManipulator::Raise(){
  current_position=manipulator_position::RAISED;
}
void HatchManipulator::Lower(){
  current_position=manipulator_position::LOWERED;
}