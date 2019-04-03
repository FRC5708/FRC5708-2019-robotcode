/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Subsystem.h>
#include <frc/Spark.h>
#include <frc/PWMTalonSRX.h>
#include <frc/Encoder.h>
#include "RobotMap.h"
#include "frc/Counter.h"
#include "Logger.h"
constexpr bool HATCH_CONTINUOUS_CONTROL=true;
constexpr int LOWERED_COUNT=390; //Test to find actual value!

class HatchManipulator : public frc::Subsystem {
 public:
	HatchManipulator();
	void InitDefaultCommand() override;
	void Periodic() override;
	void Raise();
	void Lower();
	void Stop();
	
	bool isMoving = false;

	enum Position : int {
		RAISED=-1,
		LOWERED=1,
		STOP=0
	};
	int current_position=STOP;
	frc::SpeedController* hatchMotor = IS_PROD ? (frc::SpeedController*) new frc::Spark(hatchManipulatorChannel) : 
(frc::SpeedController*) new frc::PWMTalonSRX(hatchManipulatorChannel); //Blame electrical.
	frc::Counter* hatch_counter=new frc::Counter(HatchCounterChannel); //Blame Git...
	int getCountChange();
	void updateTrueCount();
	double getDistance();
	int lastCount=0;
	int trueCount=0;
	int moveSign = 0;
	Logger* count_log= new Logger("Raw_Count","/home/lvuser/logging/Raw_Count_Last");
};
