#include "subsystems/McShootieTube.h"

#include <frc/Spark.h>
#include "RobotMap.h"

McShootieTube::McShootieTube() : Subsystem("Manipulator"),
leftMotor(new frc::Spark(ballManipulatorMotorLeft)), rightMotor(new frc::Spark(ballManipulatorMotorRight)) {

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