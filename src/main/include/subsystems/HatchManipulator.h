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
#include "frc/Counter.h"
constexpr bool HATCH_CONTINUOUS_CONTROL=true;
constexpr int LOWERED_COUNT=500; //Test to find actual value!
constexpr int RAISED=-1;
constexpr int LOWERED=1;
constexpr int STOP=0;
class HatchManipulator : public frc::Subsystem {
 public:
  HatchManipulator();
  void InitDefaultCommand() override;
  void Periodic() override;
  void Raise();
  void Lower();
  void Stop();
  int current_position=STOP;
  frc::SpeedController* hatchMotor = new frc::Spark(hatchManipulatorChannel);
  frc::Counter* hatch_counter=new frc::Counter(HatchCounterChannel);
  int getCountChange();
  void updateTrueCount();
  double getDistance();
  int lastCount=0;
  int trueCount=0;
};
