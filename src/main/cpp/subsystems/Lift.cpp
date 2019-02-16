#include "subsystems/Lift.h"

Lift::Lift() : frc::Subsystem("Lift") {}

void Lift::Elevate(int direction) {
    liftMotor->Set(speed*direction);
}

