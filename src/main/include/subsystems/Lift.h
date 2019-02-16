#pragma once

#include <frc/commands/Subsystem.h>
#include <frc/Spark.h>
#include "RobotMap.h"


class Lift : public frc::Subsystem {
 public:
  Lift();
  void InitDefaultCommand() override;
  void Elevate(int);

 private:
    static const double speed = 0.5; 
    frc::SpeedController* liftMotor = new frc::Spark(liftMotorChannel);
};
