#include "subsystems/ShiftieLiftie.h"
#include <iostream>

ShiftieLiftie::ShiftieLiftie() : frc::Subsystem("Lift"),
liftEncoder(LiftEncoderChannel[0], LiftEncoderChannel[1]) {

	liftEncoder.SetDistancePerPulse(1.0/360.0);

	//Uncomment if someone winds the winch the wrong way.
	//liftMotor->SetInverted(true);
}

void ShiftieLiftie::Elevate(Setpoint point) {

	switch (point) {
	case Bottom:
		// special case for when bottom button is held down
		// we want to go lower
		if (getPosition() < 4 && movePlace <= 0) {
			movePlace -= 0.1;
		}
		else movePlace = 0; 
		break;
	case Top: movePlace = INFINITY; break;
	}

	holdTicks = 0;
}

// measured positions of lift
struct { double encRev, liftHeight; } liftMap[] = {
	{ 0, 0 }
};
constexpr double posCount = sizeof(liftMap) / sizeof(double) / 2;


constexpr double spoolCircumfrence = 1.44 * M_PI;
constexpr double liftMult = 2;

constexpr double moveCoeff = spoolCircumfrence * liftMult;

// should return a value in inches
// could potentially make this more complicated, account for windings, etc.
double ShiftieLiftie::getPosition() {
	return liftEncoder.GetDistance() * moveCoeff;

	double encoderRevs = liftEncoder.GetDistance();
	for (int i = 0; i < posCount; ++i) {
		if (i + 1 == posCount || liftMap[i+1].encRev > encoderRevs) {
			double baseHeight = liftMap[i].liftHeight;
			if (i + 1 == posCount) --i; // interpolate from previous measurements

			return baseHeight + (encoderRevs - liftMap[i].encRev) *
			(liftMap[i+1].liftHeight - liftMap[i].liftHeight) /
			(liftMap[i+1].encRev - liftMap[i].encRev);
		}
	}
}
double ShiftieLiftie::getRate() {
	return liftEncoder.GetRate() * moveCoeff;

	double encoderRevs = liftEncoder.GetDistance();
	for (int i = 0; i < posCount; ++i) {
		if (i + 1 == posCount || liftMap[i+1].encRev > encoderRevs) {
			return liftEncoder.GetRate() * 
			(liftMap[i+1].liftHeight - liftMap[i].liftHeight) /
			(liftMap[i+1].encRev - liftMap[i].encRev);
		}
	}
}

constexpr double moveTolerance = 0.5; // inches away from the target point to be considered holding

constexpr double kMove = 0.1; // motor-powers per inch
constexpr double maxMoveSpeed = 24; // soft cap, in inches per second
constexpr double maxOverSpeed = 10; // in/sec over maxMoveSpeed, at which motors are set to 0
constexpr double slowDownPos = 4; // inches above bottom when we lower the max speed
constexpr double slowestMaxSpeed = 5; // in/sec when lowering slowly
constexpr int maxHoldTicks = 100; // 2 seconds
// must be still, at bottom, for 1 second to reset encoder distance
constexpr int calibrationWaitTicks = 50; 

void ShiftieLiftie::Periodic() {
	if (!LIFT_CONTINUOUS_CONTROL) {
		double position = getPosition();
		double rate = getRate();

		if (fabs(position - movePlace) < moveTolerance) holdTicks++;
		else holdTicks = 0;

		if (holdTicks < maxHoldTicks) {
			double movePower = (movePlace - position) * kMove;

			// clamp movePower to [-1, 1]
			movePower = copysign(std::min(fabs(movePower), 1.0), movePower);

			double realMaxSpeed = maxMoveSpeed;

			// slow down when approaching bottom so we don't slam into it
			if (position < slowDownPos) realMaxSpeed -= 
			std::max(position, 0.0) / slowDownPos * (maxMoveSpeed - slowestMaxSpeed);

			double overSpeed = fabs(rate) - realMaxSpeed;

			if (overSpeed > maxOverSpeed) movePower = 0;
			else if (overSpeed > 0) movePower -= overSpeed / maxOverSpeed * movePower;

			liftMotor->Set(movePower);
		}
		else liftMotor->Set(0);

		if (rate == 0) ++stillTicks;
		else stillTicks = 0;

		if (movePlace <= 0 && stillTicks >= calibrationWaitTicks) {
			liftEncoder.Reset();
			if (stillTicks == calibrationWaitTicks)
				std::cout << "Calibrated lift" << std::endl;
		}
	}
}

