/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/HatchManipulator.h"
#include <iostream>

HatchManipulator::HatchManipulator() : Subsystem("ExampleSubsystem") {
  hatch_counter->SetUpSource(HatchCounterChannel); // Probably wrong input/output.
  hatch_counter->SetUpDownCounterMode();
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
/*  updateTrueCount();
  if(current_position=STOP) return;
  if(current_position=RAISED){
    if(abs(trueCount - 0) > 10 ){
    hatchMotor->Set(-1.0);
    }
    else{
      current_position=STOP;
    }
  }
  if(current_position=LOWERED){
    if(abs(trueCount - LOWERED_COUNT) > 10){
      hatchMotor->Set(1.0);
    }else{
      current_position=STOP;
    }
  }*/
}
int HatchManipulator::getCountChange(){
  int difference = hatch_counter->Get() - lastCount;
  lastCount=hatch_counter->Get();
  return difference;
}
void HatchManipulator::updateTrueCount(){
  
  //0 equals all the way raised, increases as goes down.
  trueCount=trueCount + getCountChange() * current_position;
  std::cout << "True Count: " << trueCount << std::endl;
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
