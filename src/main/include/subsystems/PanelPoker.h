#pragma once

#include <frc/commands/Subsystem.h>
#include <frc/DoubleSolenoid.h>
#include "RobotMap.h"

class PanelPoker : public frc::Subsystem {
 public:
  PanelPoker();
  
  void Deploy();
  void Stow();

 private:
  frc::DoubleSolenoid* pokerSolenoid;
};
