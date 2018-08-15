#include "SwerveDrive.h"
/*
 * Throughout each of the source files, I tried to keep includes limited to only
 * the respective header file, and leave the includes I needed in said header file.
 * For some reason not worth investigation, the compiler fails to recognize the
 * "SwerveDriveTrain" subsystem as a class if it's included in "SwerveDrive.h".
 * There's probably a deep-rooted issue in my include methodology and compilation
 * order, but the important thing to know is that working > organization.
 */
#include "../Robot.h"

SwerveDrive::SwerveDrive() : testBot(0,0,0,0) {
	Requires(Robot::swervedrivetrain.get());
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
}

// Called just before this Command runs the first time
void SwerveDrive::Initialize() {
}

// Called repeatedly when this Command is scheduled to run
void SwerveDrive::Execute() {
	double robotAngle = 0;
	double wheelAngle = Robot::swervedrivetrain->EncoderToDegrees(RobotMap::rotationMotor0->GetSensorCollection().GetQuadraturePosition());
	testBot.UpdateLocation(robotAngle + wheelAngle, RobotMap::driveMotor0->GetSpeed());
	Robot::swervedrivetrain->Swerve(Robot::oi->GetRightStick()->GetMagnitude(), Robot::oi->GetRightStick()->GetTwist(), Robot::oi->GetRightStick()->GetDirectionDegrees());
}

// Make this return true when this Command no longer needs to run execute()
bool SwerveDrive::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void SwerveDrive::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void SwerveDrive::Interrupted() {

}
