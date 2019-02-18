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

class Lift : public frc::Subsystem {
 public:
	Lift();
	void Periodic() override;
	void Elevate(int placeId);
	
	double movePlace;

	double getPosition();
	double getRate();

 private:
	frc::SpeedController* liftMotor = new frc::Spark(liftMotorChannel);
	frc::Encoder liftEncoder;

	int holdTicks;
	int stillTicks;
};