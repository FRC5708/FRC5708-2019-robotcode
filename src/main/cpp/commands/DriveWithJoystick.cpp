#include "commands/DriveWithJoystick.h"
#include <iostream>
#include <frc/commands/CommandGroup.h>
#include "subsystems/HatchManipulator.h"

// buttons on xbox:
// 1=A, 2=B, 3=X, 4=Y, 5=left bumper, 6=right bumper, 7=Back, 8=Start, 9=left joystick, 10=right joystick


constexpr int INTAKE_BUTTON = 5, SHOOT_BUTTON = 6;


double currentLeftPower=0.0;
double currentRightPower=0.0;
DriveWithJoystick::DriveWithJoystick()
    : frc::Command("DriveWithJoystick"){
	Requires(&Robot::drivetrain);
}
double inputTransform(double input, double minPowerOutput, double inputDeadZone, 
		 double inputChangePosition = 0.75, double outputChangePosition = 0.5) {

	double output = 0;
	double correctedInput = (fabs(input) - inputDeadZone) / (1 - inputDeadZone);
	
	if (correctedInput <= 0) {
		return 0;
	}
	else if (correctedInput <= inputChangePosition) {
		output = (correctedInput / inputChangePosition * (outputChangePosition - minPowerOutput)) + minPowerOutput;
	}
	else {
		output = (correctedInput - inputChangePosition)
				/ (1 - inputChangePosition)
				* (1 - outputChangePosition)
				+ outputChangePosition;
	}
	if (input < 0) output = -output;
	return output;
}

void powerRampup(double input, double* outputVar) {
	if ((fabs(input) < fabs(*outputVar)) && ((input < 0 && *outputVar < 0 ) || (input > 0 && *outputVar > 0))){
		*outputVar = input;
		return;
	} 
	int sign = (input > 0) ? 1 : -1;
	*outputVar += 0.1*sign;
	
}

void cancelCommand(frc::Command* toCancel) {
	if (toCancel->GetGroup() == nullptr) {

		// cancels command, or its parent commandGroup
		toCancel->Cancel();
	}
	else cancelCommand(toCancel->GetGroup());
}
void doHatch(){
	if(!HATCH_CONTINUOUS_CONTROL){
		
	}else{
		//double power = inputTransform(Robot::driveJoystick->GetRawAxis(5), 0, 0.1, 0, 0);
		double power = 0;

		if (Robot::liftJoystick->GetRawButton(7) || Robot::driveJoystick->GetRawButton(7)) {
			power = -1;
		}
		if (Robot::liftJoystick->GetRawButton(8) || Robot::driveJoystick->GetRawButton(8)) {
			power = 1;
		}
		Robot::hatch.hatchMotor->Set(power);
	}
}
void doLiftManipulator() {
	
	int pov = Robot::liftJoystick->GetPOV();
	if (pov != -1) {
		ShiftieLiftie::Setpoint setpoint = ShiftieLiftie::Setpoint::Stay;
		if (pov == 180) setpoint = ShiftieLiftie::Setpoint::Bottom;

		if (Robot::liftJoystick->GetRawButton(1)) {
			// ball manipulator position
			switch (pov) {
				case 270: setpoint = ShiftieLiftie::Setpoint::LowGoalCargo; break;
				case 90: setpoint = ShiftieLiftie::Setpoint::MidGoalCargo; break;
				case 0: setpoint = ShiftieLiftie::Setpoint::HighGoalCargo; break;
			}
		}
		if (Robot::liftJoystick->GetRawButton(4)) {
			// hatch manipulator position
			switch (pov) {
				case 270: setpoint = ShiftieLiftie::Setpoint::LowGoalHatch; break;
				case 90: setpoint = ShiftieLiftie::Setpoint::MidGoalHatch; break;
				case 0: setpoint = ShiftieLiftie::Setpoint::HighGoalHatch; break;
			}
		}

		Robot::lift.Elevate(setpoint);
	}	

	else {
		// right stick vertical
		double power = inputTransform(Robot::liftJoystick->GetRawAxis(5), 0, 0.1, 0, 0);
		Robot::lift.MoveMotor(power);
	}

	if (Robot::liftJoystick->GetRawButton(SHOOT_BUTTON) ||
	 Robot::driveJoystick->GetRawButton(SHOOT_BUTTON)) Robot::manipulator.Shoot();
	else if (Robot::liftJoystick->GetRawButton(INTAKE_BUTTON) || 
	Robot::driveJoystick->GetRawButton(INTAKE_BUTTON)) Robot::manipulator.Intake();
	else Robot::manipulator.Stop();
}

// Called repeatedly when this Command is scheduled to run
void DriveWithJoystick::Execute() {
	double turn = 0;
	double power = 0;

	switch (joyMode){
		case SINGLE_JOY: {
			turn = -Robot::driveJoystick->GetX();
			power = Robot::driveJoystick->GetY();
			turn = inputTransform(turn, 0, 0);
			break;
		}
		case XBOX: {
			turn = Robot::driveJoystick->GetX();
			power = Robot::driveJoystick->GetRawAxis(3)-Robot::driveJoystick->GetRawAxis(2);
			turn = inputTransform(turn, 0, 0.1);

			doLiftManipulator();
			doHatch();
			break;
		}
	}
	power = inputTransform(power, 0.15, 0.03);

	if (Robot::autoDrive.commandUsing != nullptr) {
		if (fabs(power) < 0.3 && fabs(turn) < 0.3) return;
		else {
			std::cout << "cancelling auto drive" << std::endl;
			cancelCommand(Robot::autoDrive.commandUsing);
		} 
	}

	//Robot::drivetrain.DrivePolar(power, turn);
	double v = (1-fabs(turn)) * (power) + power;
	double w = (1-fabs(power)) * (turn) + turn;
	double right = (v+w)/2;
	double left = (v-w)/2;
	
	//powerRampup(left, &currentLeftPower);
	//powerRampup(right, &currentRightPower);
	
	Robot::drivetrain.Drive(left, right);
}

// Make this return true when this Command no longer needs to run execute()
bool DriveWithJoystick::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void DriveWithJoystick::End() {
	Robot::drivetrain.Drive(0, 0);
}
