#include "subsystems/McShootieTube.h"

#include <frc/Spark.h>
#include <frc/PWMTalonSRX.h>
#include "RobotMap.h"

#define CONTROLLER frc::PWMTalonSRX
//#define CONTROLLER frc::Spark

McShootieTube::McShootieTube() : Subsystem("Manipulator"),
leftMotor(new CONTROLLER(ballManipulatorMotorLeft)), rightMotor(new CONTROLLER(ballManipulatorMotorRight)) {

	// Change this if manipulator controls are inverted
	rightMotor->SetInverted(true);
}

void McShootieTube::Intake() {
	leftMotor->Set(-1);
	rightMotor->Set(-1);
}
void McShootieTube::Shoot() {
	leftMotor->Set(1);
	rightMotor->Set(1);
}
void McShootieTube::Stop() {
	leftMotor->StopMotor();
	rightMotor->StopMotor();
}