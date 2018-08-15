/*
 * ZeroWheels.cpp
 *
 *  Created on: May 26, 2018
 *      Author: brrobotics5162
 */


#include "ZeroWheels.h"
#include "../../Robot.h"
#include "../../RobotMap.h"

ZeroWheels::ZeroWheels() {
	Requires(Robot::swervedrivetrain.get());
}

// Called just before this Command runs the first time
void ZeroWheels::Initialize() {
}

// Called repeatedly when this Command is scheduled to run
void ZeroWheels::Execute() {
	// I'm lazy.
	Robot::swervedrivetrain->ZeroWheels();
}

// Make this return true when this Command no longer needs to run execute()
bool ZeroWheels::IsFinished() {
	return true;
}

// Called once after isFinished returns true
void ZeroWheels::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ZeroWheels::Interrupted() {

}

