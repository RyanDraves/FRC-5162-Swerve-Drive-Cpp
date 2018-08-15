/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <AHRS.h>
#include <SmartDashboard/SendableChooser.h>
#include <TimedRobot.h>

#include <Commands/Command.h>
#include "Commands/Auton.h"

#include "Subsystems/SwerveDriveTrain.h"
#include "OI.h"
#include "RobotMap.h"

class Robot : public frc::TimedRobot {
public:
	static std::unique_ptr<SwerveDriveTrain> swervedrivetrain;
	static std::unique_ptr<OI> oi;

	void RobotInit() override;
	void DisabledInit() override;
	void DisabledPeriodic() override;
	void AutonomousInit() override;
	void AutonomousPeriodic() override;
	void TeleopInit() override;
	void TeleopPeriodic() override;
	void TestPeriodic() override;

private:
	std::unique_ptr<Command> auton;
};
