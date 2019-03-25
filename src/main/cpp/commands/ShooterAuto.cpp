/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/ShooterAuto.h"
#include "subsystems/ShiftieLiftie.h"
#include "Commands/McShootieAuto.h"
#include "Robot.h"
#include "Robotmap.h"

ShooterAuto::ShooterAuto(ShiftieLiftie::Setpoint setpoint) {
  this->setpoint=setpoint;
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
}

// Called just before this Command runs the first time
void ShooterAuto::Initialize() {
this->liftMove=new LiftMove(this->setpoint);
}

// Called repeatedly when this Command is scheduled to run
void ShooterAuto::Execute() {
  if(this->liftMove->IsFinished() && this->ballShoot!=nullptr){
    this->ballShoot=new BallShoot();
  }
}

// Make this return true when this Command no longer needs to run execute()
bool ShooterAuto::IsFinished() { return this->liftMove->IsFinished() && this->ballShoot->IsFinished(); }

// Called once after isFinished returns true
void ShooterAuto::End() {
  delete this->liftMove;
  delete this->ballShoot;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ShooterAuto::Interrupted() {
  this->ballShoot->Cancel();
}
void ShooterAuto::Cancel(){
  this->ballShoot->Cancel();
}