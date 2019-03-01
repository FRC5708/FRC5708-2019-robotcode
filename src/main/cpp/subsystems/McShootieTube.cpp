#include "subsystems/McShootieTube.h"

#include <frc/Spark.h>
#include <frc/PWMTalonSRX.h>
#include "RobotMap.h"


McShootieTube::McShootieTube() : Subsystem("Manipulator"),
leftMotor(IS_PROD ? (frc::SpeedController*) new frc::Spark(ballManipulatorMotorLeft) : (frc::SpeedController*) new frc::PWMTalonSRX(ballManipulatorMotorLeft)), rightMotor(IS_PROD ? (frc::SpeedController*) new frc::Spark(ballManipulatorMotorRight) : (frc::SpeedController*) new frc::PWMTalonSRX(ballManipulatorMotorRight)) {

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