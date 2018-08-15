#include "SwerveDriveTrain.h"
#include <Math.h>

/*
 * The PID values seem to best work with a medium P value, very small I value,
 * and large value.
 */

#define MAX_ROT_SPEED 0.5 // Maximum Rotational Speed
#define P_VAL 3			  // P value of PID loop
#define I_VAL 0.1			  // I value of PID loop
#define D_VAL 6			  // D value of PID loop
// 0.3 6 0.25 3 works great
// 0.5 3 0.1 6 works great

#define SPEED_THROTTLE 3 // It's too loud sometimes. Make this 1 later, it divides
						 // the drive speed by this number.

SwerveDriveTrain::SwerveDriveTrain() : Subsystem("DriveTrain") {
	wheelAngle = 0;
	wheelOffset = 0;
	driveSpeeds.clear();
	twistSpeeds.clear();
	rotSpeeds.clear();
	fieldOriented = true;

	// PID loop variables
	prevUnits = 5162; // Some abstract number so the first loop does something.
	prevTarget = 0;
	rotationPID = new PID(P_VAL, I_VAL, D_VAL, 100, -100);
	estimatedWheelAngle = 0;
	// 		  arbitrary
	multiplier = 15;

	// Value for determining if the wheel angle reading is fresh or not.
	oldVal = 0;

	// Debugging values
	lastJ = 0;
	lastWA = 0;
	correctionSum = 0;

	// Test
	RobotMap::rotationMotor0->SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_3_Quadrature, 20, 0);
}

void SwerveDriveTrain::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	SetDefaultCommand(new SwerveDrive());
}

void SwerveDriveTrain::Swerve(double speed, double twist, double rotAngle) {
	int units = RobotMap::rotationMotor0->GetSensorCollection().GetQuadraturePosition();
	wheelAngle = EncoderToDegrees(units);
	wheelAngle = fmod(wheelAngle + wheelOffset, 360);
	if (wheelAngle < 0) {
		wheelAngle += 360;
	}

	SmartDashboard::PutNumber("rot", rotAngle);
	// Invert the rotation angle because the rotation motor is upside down.
	rotAngle -= 2 * rotAngle;

	// The rotational adjustment may invert the wheels on a whim, so that's calculated first.
	rotSpeeds = AdjustForRotation(rotAngle, speed, (units != prevUnits));
	prevUnits = units;
	driveSpeeds = AdjustForSpeed(speed);
	twistSpeeds = AdjustForTwist(twist);

	SetDriveMotors(driveSpeeds, twistSpeeds);
	SetRotationMotors(rotSpeeds);

	SmartDashboard::PutNumber("Wheel Angle", wheelAngle);
}

std::vector<double> SwerveDriveTrain::AdjustForSpeed(double speed) {
	if (speed < 0.1) {
		return SetVector(0);
	}
	else {
		return SetVector(speed);
	}
}

/*
 * To turn the robot, and not the wheels, one side needs to be faster than the other.
 * Using the angle of the wheels, we determine which side is the "right side" of wheels
 * dynamically, and adjust the speed based on that.
 */
std::vector<double> SwerveDriveTrain::AdjustForTwist(double twist) {
	/*
	 * This uses only one wheel's angle, which could be dangerous.
	 * But to be honest, whatcha gonna do if the wheels aren't aligned?
	 * You were screwed anyways.
	 * To be consistent, wheel 0 will always be called on when only one angle is used.
	 */
	double radians = (wheelAngle * M_PI) / 180.0;
	std::vector<double> speeds;
	speeds.push_back(cos(radians - (M_PI / 4.0)) * twist);
	speeds.push_back(sin(radians - (M_PI / 4.0)) * twist);
	speeds.push_back(cos(radians + (M_PI / 4.0)) * twist);
	speeds.push_back(sin(radians - (3 * M_PI / 4.0)) * twist);
	return speeds;
	/*
	 * If statements go around the robot as such:
	 *
	 * 		Front
	 * 	  315_0_45			Reference:
	 * 		|	|			  Front
	 * 	 270|	|90			  0  1
	 * 		|___|			  2  3
	 * 	 225 180 135		  Back
	 * 		Back
	 *
	 * They're more for documentation to help lay out the trigonometry.
	 */
	/*if (0) {
		speeds[0] += twist;
		speeds[1] += -twist;
		speeds[2] += twist;
		speeds[3] += -twist;
	}
	else if (45) {
		speeds[0] += 2*twist;
		speeds[1] += 0*twist;
		speeds[2] += 0*twist;
		speeds[3] += -2*twist;
	}
	else if (90) {
		speeds[0] += twist;
		speeds[1] += twist;
		speeds[2] += -twist;
		speeds[3] += -twist;
	}
	else if (135) {
		speeds[0] += 0*twist;
		speeds[1] += 2*twist;
		speeds[2] += -2*twist;
		speeds[3] += 0*twist;
	}
	else if (180) {
		speeds[0] += -twist;
		speeds[1] += twist;
		speeds[2] += -twist;
		speeds[3] += twist;
	}
	else if (225) {
		speeds[0] += -2*twist;
		speeds[1] += 0*twist;
		speeds[2] += 0*twist;
		speeds[3] += 2*twist;
	}
	else if (270) {
		speeds[0] += -twist;
		speeds[1] += -twist;
		speeds[2] += twist;
		speeds[3] += twist;
	}
	else if (315) {
		speeds[0] += 0*twist;
		speeds[1] += -2*twist;
		speeds[2] += 2*twist;
		speeds[3] += 0*twist;
	}*/
}

std::vector<double> SwerveDriveTrain::AdjustForRotation(double rotAngle, double speed, bool newValues) {
	std::vector<double> speeds;

	/*
	 * New wheel values are set to come in every 20ms, the speed of the code.
	 * Sometimes this doesn't occur (although rarely), so after extensive testing
	 * when it would only come in every 160ms, a wheel estimate is made for the
	 * loops where a new encoder value has not been read.
	 *
	 * The estimated wheel angle is figured by a fairly arbitrary number.
	 * It's not GOOD, it gets the job done.
	 */
	// If for some reason Talon's default 160ms isn't changed, it's once every
	// 8 loops and will function "ok" with estimation.
	if (newValues || oldVal == 7) {
		// For debugging purposes, I'm not going through the logic of
		// wrapped and normal angles for good values. Skip it if it's too big.
		estimatedWheelAngle += rotationPID->GetCorrection() * multiplier;
		if (estimatedWheelAngle - wheelAngle < 100 && estimatedWheelAngle - wheelAngle > -100) {
			SmartDashboard::PutNumber("Estimate Error", (estimatedWheelAngle - wheelAngle));
			if (correctionSum != 0 && (wheelAngle - lastWA) / correctionSum < 50 && (wheelAngle - lastWA) / correctionSum > 0) {
				SmartDashboard::PutNumber("deltaWA to C", ((wheelAngle - lastWA) / correctionSum));
			}
			else {
				SmartDashboard::PutNumber("deltaWA to C", 0);
			}
		}
		correctionSum = 0;
		if (oldVal != 7) {
			SmartDashboard::PutNumber("Early", SmartDashboard::GetNumber("Early", 0) + 1);
			SmartDashboard::PutNumber("oldVal", oldVal);
		}
		oldVal = 0;
		estimatedWheelAngle = wheelAngle;
	}
	else {
		oldVal++;
		//												practically arbitrary
		estimatedWheelAngle += rotationPID->GetCorrection() * multiplier;
		lastWA = wheelAngle;
		correctionSum += rotationPID->GetCorrection();
	}

	// Establish a 10% minimum speed before rotation kicks in.
	// This prevents wild degree changes as the joystick begins to move.
	// Have a button slow the robot down for finer motor control.
	// 2018's robot had a button to hold for ~50% speed.
	if (speed < 0.1) {
		speeds = SetVector(0);
		rotationPID->SetCorrection(0);
		return speeds;
	}

	/*
	 * Some logic requires absolute angle difference, so a modified duplicate of
	 * the angles will be used to figure it out between both angle measurements.
	 *	 Wrapped		   Normal
	 * 		0			     0
	 * 	-90 + 90 	&	 270 + 90
	 * 	  +-180			    180
	 */
	double wrappedWheelAngle = GetWrappedAngle(estimatedWheelAngle);
	double wrappedRotAngle = rotAngle;

	// This chunk makes all wheel rotations less than or equal to 135 degrees,
	// inverting the motors as needed to do so.
	if (fabs(GetNormalAngle(rotAngle) - estimatedWheelAngle) > 135 && fabs(wrappedWheelAngle - wrappedRotAngle) > 135) {
		// Invert the drive motors, and make the turn much smaller.
		RobotMap::driveMotor0->SetInverted(!RobotMap::driveMotor0->GetInverted());
		RobotMap::driveMotor1->SetInverted(!RobotMap::driveMotor1->GetInverted());
		RobotMap::driveMotor2->SetInverted(!RobotMap::driveMotor2->GetInverted());
		RobotMap::driveMotor3->SetInverted(!RobotMap::driveMotor3->GetInverted());

		if (estimatedWheelAngle >= 180) {
			estimatedWheelAngle -= 180;
		}
		else {
			estimatedWheelAngle += 180;
		}
		wrappedWheelAngle = GetWrappedAngle(estimatedWheelAngle);

		wheelOffset = fmod(wheelOffset + 180, 360);
	}

	// All angles are now less than 135 degrees, so whichever difference matches
	// that is the correction angle difference between the wheels.
	// (359 to 1 degree should be a difference of 2, not 358.
	double normalDifference = GetNormalAngle(rotAngle) - estimatedWheelAngle;
	double wrappedDifference = wrappedRotAngle - wrappedWheelAngle;

	/*
	 * PID loop begins
	 *
	 * The bulk of the code is housed in the PID class.
	 * Here we just have to determine the error and
	 * whether or not to reset the cumulative error
	 * (moving to a new target/overshoot on the correction).
	 */

	// Set error between -100 and 100
	double error;
	if (normalDifference <= 135 && normalDifference >= -135) {
		error = normalDifference * (100.0 / 135.0);
	}
	else {
		error = wrappedDifference * (100.0 / 135.0);
	}
	SmartDashboard::PutNumber("PID Error", error);

	// Reset the cumulative error if the target has changed
	// or if the error's sign has flipped.
	if (rotAngle != prevTarget) {
		rotationPID->ResetCumulativeError();
	}
	prevTarget = rotAngle;
	if ((rotationPID->GetPreviousError() < 0 && error > 0) || (error < 0 && rotationPID->GetPreviousError() > 0)) {
		rotationPID->ResetCumulativeError();
	}

	rotationPID->PIDWrite(error);

	// Scale -100 to 100 down to -MAX_ROT_SPEED to MAX_ROT_SPEED.
	rotationPID->SetCorrection(rotationPID->GetCorrection() * MAX_ROT_SPEED / 100.0);
	speeds = SetVector(rotationPID->GetCorrection());
	SmartDashboard::PutNumber("Correction", rotationPID->GetCorrection());
	return speeds;
}

void SwerveDriveTrain::SetDriveMotors(std::vector<double> driveSpeeds, std::vector<double> twistSpeeds) {
	/*
	 * Take the driveSpeeds and merge them "gracefully" into the twistSpeeds.
	 */
	std::vector<double> finalSpeeds;

	for (unsigned int i = 0; i < 4; i++) {
		finalSpeeds.push_back(driveSpeeds[i] + twistSpeeds[i]);
	}

	double maxSpeed = 1;
	for (double speed : finalSpeeds) {
		if (speed > 1) {
			maxSpeed = speed;
		}
	}
	for (double speed : finalSpeeds) {
		speed /= maxSpeed;
	}
	//RobotMap::driveMotor0->Set(ControlMode::PercentOutput, finalSpeeds[0]);
	RobotMap::driveMotor0->Set(finalSpeeds[0] / SPEED_THROTTLE);
	// VictorSP's
	RobotMap::driveMotor1->Set(finalSpeeds[1] / SPEED_THROTTLE);
	RobotMap::driveMotor2->Set(finalSpeeds[2] / SPEED_THROTTLE);
	RobotMap::driveMotor3->Set(finalSpeeds[3] / SPEED_THROTTLE);
	Wait(0.005);
}
void SwerveDriveTrain::SetRotationMotors(std::vector<double> rotSpeeds) {
	RobotMap::rotationMotor0->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, rotSpeeds[0]);
	RobotMap::rotationMotor1->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, rotSpeeds[1]);
	RobotMap::rotationMotor2->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, rotSpeeds[2]);
	RobotMap::rotationMotor3->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, rotSpeeds[3]);
}

void SwerveDriveTrain::ZeroWheels() {
	wheelOffset -= wheelAngle;
}

void SwerveDriveTrain::ZeroYaw() {
	RobotMap::ahrs->ZeroYaw();
}

double SwerveDriveTrain::EncoderToDegrees(double units) {
	// Don't trust this number at all. Do your own testing.
	return fmod(0.0879 * units, 360);
}

double SwerveDriveTrain::EncoderToInches(double units) {
	// Don't trust this number at all. Do your own testing.
	return 0.00621 * units;
}

double SwerveDriveTrain::GetNormalAngle(double angle) {
	//Converts -180 to 180 angles into 0 to 360.
	double normalAngle = angle;
	if (normalAngle < 0) {
		normalAngle += 360;
	}
	return normalAngle;
}

double SwerveDriveTrain::GetWrappedAngle(double angle) {
	// Converts 0 to 360 angles into -180 to 180.
	double wrappedAngle = angle;
	if (wrappedAngle > 180) {
		wrappedAngle -= 360;
	}
	if (wrappedAngle < -180) {
		wrappedAngle += 360;
	}
	return wrappedAngle;
}

std::vector<double> SwerveDriveTrain::SetVector(double speed) {
	std::vector<double> speeds;
	for (unsigned int i = 0; i < 4; i++) {
		speeds.push_back(speed);
	}
	return speeds;
}

bool SwerveDriveTrain::GetFieldOriented() {
	return fieldOriented;
}
void SwerveDriveTrain::SetFieldOriented(bool val) {
	fieldOriented = val;
}
