#pragma once

#include <frc/commands/CommandGroup.h>
#include "subsystems/AutoDrive.h"

class Autonomous : public frc::CommandGroup {
public:
	Autonomous(std::vector<AutoDrive::Point> points);
};