#include "commands/DriveWithJoystick.h"

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
	if (fabs(input) < fabs(*outputVar) && ((input > 0 && *outputVar > 0) | (input < 0 && *outputVar < 0))){
		 *outputVar = input;
		 return;
	}
	int sign = (input > 0) ? 1 : -1;
	*outputVar += 0.1*sign;
}

void doLiftManipulator() {
	if(!LIFT_CONTINUOUS_CONTROL) {
		int pov = Robot::joystick->GetPOV();
		if (pov != -1) {
			ShiftieLiftie::Setpoint setpoint;
			switch (pov) {
				case 0: setpoint = ShiftieLiftie::Setpoint::Top; break;
				case 180: setpoint = ShiftieLiftie::Setpoint::Bottom; break;
			}
			Robot::lift.Elevate(setpoint);
		}
	}
	else {
		// I think this is the left stick vertical
		double power = inputTransform(Robot::joystick->GetRawAxis(5), 0, 0.1, 0, 0);
		Robot::lift.liftMotor->Set(power);
	}

	if (Robot::joystick->GetRawButton(SHOOT_BUTTON)) Robot::manipulator.Shoot();
	else if (Robot::joystick->GetRawButton(INTAKE_BUTTON)) Robot::manipulator.Intake();
	else Robot::manipulator.Stop();
}

// Called repeatedly when this Command is scheduled to run
void DriveWithJoystick::Execute() {
	double turn = 0;
	double power = 0;

	switch (joyMode){
		case SINGLE_JOY: {
			turn = -Robot::joystick->GetX();
			power = Robot::joystick->GetY();
			turn = inputTransform(turn, 0, 0);
			break;
		}
		case XBOX: {
			turn = Robot::joystick->GetX();
			power = Robot::joystick->GetRawAxis(3)-Robot::joystick->GetRawAxis(2);
			turn = inputTransform(turn, 0, 0.1);

			doLiftManipulator();
			break;
		}
	}
	power = inputTransform(power, 0.2, 0.05);

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
