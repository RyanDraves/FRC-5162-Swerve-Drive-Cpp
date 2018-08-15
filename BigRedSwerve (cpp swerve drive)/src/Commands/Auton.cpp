/*
 * Auton.cpp
 *
 *  Created on: May 26, 2018
 *      Author: brrobotics5162
 */

#include "Auton.h"
#include "../Robot.h"

#define AUTON_SPEED 1

Auton::Auton(double startX, double startY, double endX, double endY) : robot(startX, startY, endX, endY) {
	Requires(Robot::swervedrivetrain.get());
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
}

// Called just before this Command runs the first time
void Auton::Initialize() {
}

// Called repeatedly when this Command is scheduled to run
void Auton::Execute() {
	//double robotAngle = RobotMap::ahrs->GetYaw();
	double robotAngle = 0;
	double wheelAngle = Robot::swervedrivetrain->EncoderToDegrees(RobotMap::rotationMotor0->GetSensorCollection().GetQuadraturePosition());
	robot.UpdateLocation(robotAngle + wheelAngle, RobotMap::driveMotor0->GetSpeed());
	double test = robot.GetNextAngle();
	SmartDashboard::PutNumber("Next Angle", test);
	Robot::swervedrivetrain->Swerve(AUTON_SPEED, robot.GetNextTwist(), test);
}

// Make this return true when this Command no longer needs to run execute()
bool Auton::IsFinished() {
	return robot.IsFinished();
}

// Called once after isFinished returns true
void Auton::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void Auton::Interrupted() {}
