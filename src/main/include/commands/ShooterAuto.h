/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Command.h>
#include "subsystems/ShiftieLiftie.h"
#include "commands/McShootieAuto.h"

class ShooterAuto : public frc::Command {
 public:
  ShiftieLiftie::Setpoint setpoint;
  LiftMove* liftMove=nullptr;
  BallShoot* ballShoot=nullptr;
  ShooterAuto(ShiftieLiftie::Setpoint);
  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End() override;
  void Interrupted() override;
  void Cancel();
};
