#pragma once

#include <frc/commands/Subsystem.h>
#include <frc/Spark.h>
#include <frc/Encoder.h>
#include "RobotMap.h"


/* THINGS TO CHECK FOR BEFORE USING THIS CODE:

Encoder type and DIRECTION
Motor direction
Spool diameter and/or rope winding function
Make sure the limit switches are in place
Do SPARKS stay in brake mode when limit switches are activated??

*/

// if true, the lift will be controlled by a joystick instead of at discrete intervals
constexpr bool LIFT_CONTINUOUS_CONTROL = true;

class ShiftieLiftie : public frc::Subsystem {
 public:
	enum Setpoint {
		Top,
		Bottom
	};

	ShiftieLiftie();
	void Periodic() override;
	void Elevate(Setpoint setpoint);
	
	double movePlace;

	double getPosition();
	double getRate();


	frc::SpeedController* liftMotor = new frc::Spark(liftMotorChannel);
 private:
	frc::Encoder liftEncoder;

	int holdTicks;
	int stillTicks;
};