/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Subsystem.h>
#include <frc/Spark.h>
#include <frc/Encoder.h>
#include "RobotMap.h"
constexpr bool HATCH_CONTINUOUS_CONTROL=true;
class HatchManipulator : public frc::Subsystem {
 private:
  // It's desirable that everything possible under private except
  // for methods that implement subsystem capabilities

 public:
  HatchManipulator();
  void InitDefaultCommand() override;
  void Raise();
  void Lower();
  enum manipulator_position{RAISED,LOWERED};
  manipulator_position current_position=manipulator_position::RAISED;
  frc::SpeedController* hatchMotor = new frc::Spark(hatchManipulatorChannel);
  
};
