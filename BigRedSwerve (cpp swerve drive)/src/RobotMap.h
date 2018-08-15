/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

// For example to map the left and right motors, you could define the
// following variables to use with your drivetrain subsystem.
// constexpr int kLeftMotor = 1;
// constexpr int kRightMotor = 2;

// If you are using multiple modules, make sure to define both the port
// number and the module. For example you with a rangefinder:
// constexpr int kRangeFinderPort = 1;
// constexpr int kRangeFinderModule = 1;

#include "ctre/Phoenix.h"
#include "WPILib.h"
#include "Encoder.h"
#include "AHRS.h"
#include "LiveWindow/LiveWindow.h"


class RobotMap {
public:
	static void init();

	static std::shared_ptr<AHRS> ahrs;

	/*
	 * 	 Front
	 *   0	1
	 *   2 	3
	 *   Back
	 *
	 * Wish this was TalonSRX but we only have 5 :(
	 * The one TalonSRX is the only distance encoder. RIP math calculations.
	 */
	static std::shared_ptr<VictorSP> driveMotor0;
	static std::shared_ptr<VictorSP> driveMotor1;
	static std::shared_ptr<VictorSP> driveMotor2;
	static std::shared_ptr<VictorSP> driveMotor3;


	static std::shared_ptr<WPI_TalonSRX> rotationMotor0;
	static std::shared_ptr<WPI_TalonSRX> rotationMotor1;
	static std::shared_ptr<WPI_TalonSRX> rotationMotor2;
	static std::shared_ptr<WPI_TalonSRX> rotationMotor3;

	static std::shared_ptr<DifferentialDrive> robot_drive;
};
