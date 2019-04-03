/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/HatchManipulator.h"
#include <iostream>
#include "Logger.h"

HatchManipulator::HatchManipulator() : Subsystem("Hatch") {
  lastCount=hatch_counter->Get();
  trueCount=0;
  hatchMotor->SetInverted(true);
}

void HatchManipulator::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
}/*
void HatchManipulator::Periodic(){
	
}
// Put methods for controlling this subsystem
// here. Call these from Commands.
void HatchManipulator::Raise(){
	current_position=manipulator_position::RAISED;
}*/
void HatchManipulator::Periodic(){
	updateTrueCount();
	if (current_position == RAISED && trueCount > 0) {
		hatchMotor->Set(-1.0);
	}
	else if (current_position == LOWERED && trueCount < LOWERED_COUNT) {
		hatchMotor->Set(1.0);
	}
	else {
		hatchMotor->Set(0);
	}
}
int HatchManipulator::getCountChange(){
	int curr_count=hatch_counter->Get();
	int difference = curr_count - lastCount;
	lastCount=curr_count;
	return difference;
}
void HatchManipulator::updateTrueCount(){
	static int timer = 0;
	++timer;

	double motorPower = hatchMotor->Get();
	if (motorPower != 0) {
		moveSign = (motorPower > 0) ? 1 : -1;
	}
	if (motorPower==0) moveSign=0; //Maybe stops weird race condition?
	//0 equals all the way raised, increases as goes down.
	trueCount=trueCount + getCountChange() * moveSign;

	if (timer % 10 == 0) {
		std::cout << "True Count: " << trueCount << " raw count: "<< hatch_counter->Get() << std::endl;
	}
	count_log->log(std::to_string(hatch_counter->Get()).c_str());
}
void HatchManipulator::Lower(){
	current_position=LOWERED;
}
void HatchManipulator::Raise(){
	current_position=RAISED;
}
void HatchManipulator::Stop(){
	current_position=STOP;
}
