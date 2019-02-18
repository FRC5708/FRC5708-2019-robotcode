#pragma once

#include <frc/commands/Subsystem.h>
#include <frc/SpeedController.h>

class McShootieTube : public frc::Subsystem {
public:
	McShootieTube();

	void Intake();
	void Shoot();
	void Stop();

private:
	
	frc::SpeedController *leftMotor, *rightMotor;
};
