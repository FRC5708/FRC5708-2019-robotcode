#include "subsystems/ShiftieLiftie.h"
#include <iostream>

ShiftieLiftie::ShiftieLiftie() : frc::Subsystem("Lift"),
liftEncoder(LiftEncoderChannel[0], LiftEncoderChannel[1]) {

	liftEncoder.SetDistancePerPulse(1.0/20.0);
	liftEncoder.SetSamplesToAverage(4);

	//Uncomment if someone winds the winch the wrong way.
	//liftMotor->SetInverted(true);
}


// must be still, at bottom, for 1 second to reset encoder distance
constexpr int calibrationWaitTicks = 50; 

void ShiftieLiftie::Elevate(Setpoint point) {

// Height of the manipulator above the ground when the lift is at its lowest point
constexpr double shootieZero = 6.5;
constexpr double hatchZero = 1*12+7;

	switch (point) {
	case LowGoalHatch:
	case MidGoalHatch:
	case Bottom:
		// special case for when bottom button is held down
		// we want to go lower
		if (getPosition() < 4 && movePlace <= 0) {
			movePlace -= 0.1;
		}
		else movePlace = 0; 
		break;
	case Top: movePlace = INFINITY; break;
	// ROCKET low goal
	case LowGoalCargo: movePlace = 2*12+3.5 - shootieZero; break;
	// CARGO SHIP goal
	case MidGoalCargo: movePlace = 2*12+7.5 + 9 - shootieZero; break;
	// ROCKET middle goal
	case HighGoalCargo: movePlace = 2*12+3.5 + 2*12+4 - shootieZero; break;
	case HighGoalHatch: movePlace = 2*12+4; break;
	case Stay: break;
	}

	holdTicks = 0;
	stillTicks = 0;

	doAutoLift = true;
}

void ShiftieLiftie::MoveMotor(double power) {
	if (power != 0) {
	doAutoLift = false;
	liftMotor->Set(power);
	}
	else {
		if (!doAutoLift) liftMotor->Set(0);
	}
}

// measured positions of lift
// The data is linear enough that we don't need this
// the noise from the low-resolution encoder is larger than the spooling effect
struct { double encRev, liftHeight; } liftMap[] = {
	{ 0, 0 }
};
constexpr double posCount = sizeof(liftMap) / sizeof(double) / 2;

// determined experimentally
constexpr double moveCoeff = 9.209543;

// should return a value in inches
double ShiftieLiftie::getPosition() {
	return liftEncoder.GetDistance() * moveCoeff;
/*
	double encoderRevs = liftEncoder.GetDistance();
	for (int i = 0; i < posCount; ++i) {
		if (i + 1 == posCount || liftMap[i+1].encRev > encoderRevs) {
			double baseHeight = liftMap[i].liftHeight;
			if (i + 1 == posCount) --i; // interpolate from previous measurements

			return baseHeight + (encoderRevs - liftMap[i].encRev) *
			(liftMap[i+1].liftHeight - liftMap[i].liftHeight) /
			(liftMap[i+1].encRev - liftMap[i].encRev);
		}
	}*/
}
double ShiftieLiftie::getRate() {
	return liftEncoder.GetRate() * moveCoeff;
/*
	double encoderRevs = liftEncoder.GetDistance();
	for (int i = 0; i < posCount; ++i) {
		if (i + 1 == posCount || liftMap[i+1].encRev > encoderRevs) {
			return liftEncoder.GetRate() * 
			(liftMap[i+1].liftHeight - liftMap[i].liftHeight) /
			(liftMap[i+1].encRev - liftMap[i].encRev);
		}
	}*/
}



constexpr double moveTolerance = 0.5; // inches away from the target point to be considered holding

constexpr double kMove = 0.4; // motor-powers per inch
constexpr double maxMoveSpeed = 36; // soft cap, in inches per second
constexpr double maxOverSpeed = 10; // in/sec over maxMoveSpeed, at which motors are set to 0
constexpr double slowDownPos = 2; // inches above bottom when we lower the max speed
constexpr double slowestMaxSpeed = 10; // in/sec when lowering slowly
constexpr int maxHoldTicks = 100; // 2 seconds

bool ShiftieLiftie::isDone() {
	return fabs(getPosition() - movePlace) < moveTolerance || holdTicks > 10;
}

void ShiftieLiftie::Periodic() {
	if (doAutoLift) {
		double position = getPosition();
		double rate = getRate();

		if (fabs(position - movePlace) < moveTolerance || rate == 0) holdTicks++;
		else holdTicks = 0;

		if (holdTicks < maxHoldTicks) {
			double movePower = (movePlace - position) * kMove;

			// clamp movePower to [-1, 1]
			movePower = copysign(std::min(fabs(movePower), 1.0), movePower);

			double realMaxSpeed = maxMoveSpeed;

			// slow down when approaching bottom so we don't slam into it
			if (position < slowDownPos && movePower < 0) realMaxSpeed -= 
			std::max(position, 0.0) / slowDownPos * (maxMoveSpeed - slowestMaxSpeed);

			// slow down if we are going too fast
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

