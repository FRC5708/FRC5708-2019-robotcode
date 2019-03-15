#pragma once

#include <frc/commands/Subsystem.h>
#include <frc/Spark.h>
#include <frc/Encoder.h>
#include "RobotMap.h"


/* THINGS TO CHECK FOR BEFORE USING THIS CODE:

Do SPARKS stay in brake mode when limit switches are activated??

*/

// if true, the lift will be controlled by a joystick instead of at discrete intervals
constexpr bool LIFT_CONTINUOUS_CONTROL = false;

class ShiftieLiftie : public frc::Subsystem {
 public:
	enum Setpoint {
		Top,
		Bottom,
		LowGoalCargo,
		LowGoalHatch,
		MidGoalCargo,
		MidGoalHatch,
		HighGoalCargo,
		HighGoalHatch
	};

	ShiftieLiftie();
	void Periodic() override;
	void Elevate(Setpoint setpoint);
	
	double movePlace;

	double getPosition();
	double getRate();

	bool isDone();


	frc::SpeedController* liftMotor = new frc::Spark(liftMotorChannel);
 private:
	frc::Encoder liftEncoder;

	int holdTicks;
	int stillTicks;
};