#include "LazyFieldToggle.h"
#include "../../Robot.h"

LazyFieldToggle::LazyFieldToggle() {
	Requires(Robot::swervedrivetrain.get());
}

// Called just before this Command runs the first time
void LazyFieldToggle::Initialize() {
}

// Called repeatedly when this Command is scheduled to run
void LazyFieldToggle::Execute() {
	// I'm lazy.
	Robot::swervedrivetrain->SetFieldOriented(!Robot::swervedrivetrain->GetFieldOriented());
}

// Make this return true when this Command no longer needs to run execute()
bool LazyFieldToggle::IsFinished() {
	return true;
}

// Called once after isFinished returns true
void LazyFieldToggle::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void LazyFieldToggle::Interrupted() {

}
