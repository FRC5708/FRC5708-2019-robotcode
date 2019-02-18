#include "subsystems/Lift.h"
#include <iostream>

Lift::Lift() : frc::Subsystem("Lift"),
liftEncoder(LiftEncoderChannel[0], LiftEncoderChannel[1]) {

	liftEncoder.SetDistancePerPulse(1.0/360.0);
}

void Lift::Elevate(int placeId) {
	// Set movePlace based on placeId
	// For now, bottom is 1 and top is 2

	switch (placeId) {
	case 1:
		movePlace = -INFINITY; break;
	case 2:
		movePlace = INFINITY; break;
	}

	holdTicks = 0;
}

constexpr double spoolCircumfrence = 3; // TODO

// should return a value in inches
// could potentially make this more complicated, account for windings, etc.
double Lift::getPosition() {
	return liftEncoder.GetDistance() * spoolCircumfrence;
}
double Lift::getRate() {
	return liftEncoder.GetRate() * spoolCircumfrence;
}

constexpr double moveTolerance = 0.5; // inches away from the target point to be considered holding

constexpr double kMove = 0.1; // motor-powers per inch
constexpr double maxMoveSpeed = 24; // soft cap, in inches per second
constexpr double maxOverSpeed = 10; // in/sec over maxMoveSpeed, at which motors are set to 0
constexpr int maxHoldTicks = 100; // 2 seconds
// must be still, at bottom, for 1 second to reset encoder distance
constexpr int calibrationWaitTicks = 50; 

void Lift::Periodic() {

	if (fabs(getPosition() - movePlace) < moveTolerance) holdTicks++;
	else holdTicks = 0;

	if (holdTicks < maxHoldTicks) {
		double movePower = (movePlace - getPosition()) * kMove;

		// clamp movePower to [-1, 1]
		movePower = copysign(std::min(fabs(movePower), 1.0), movePower);

		double overSpeed = fabs(getRate()) - maxMoveSpeed;

		if (overSpeed > maxOverSpeed) movePower = 0; // activate brake mode
		else if (overSpeed > 0) movePower -= overSpeed / maxOverSpeed * movePower;

		liftMotor->Set(movePower);
	}
	else liftMotor->Set(0);

	if (getRate() == 0) ++stillTicks;
	else stillTicks = 0;

	if (movePlace == -INFINITY && stillTicks >= calibrationWaitTicks) {
		liftEncoder.Reset();
		if (stillTicks == calibrationWaitTicks)
			std::cout << "Calibrated lift" << std::endl;
	}
}

