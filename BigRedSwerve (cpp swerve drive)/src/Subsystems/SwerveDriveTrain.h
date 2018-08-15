/*
 * SwerveDriveTrain.h
 *
 *  Created on: Mar 28, 2018
 *      Author: RyanTheRat
 */

#ifndef SRC_SUBSYSTEMS_SWERVEDRIVETRAIN_H_
#define SRC_SUBSYSTEMS_SWERVEDRIVETRAIN_H_


#include <Commands/Subsystem.h>
#include "../RobotMap.h"
#include "Commands/SwerveDrive.h"
#include "PID/PID.h"
#include <iostream>

class SwerveDriveTrain : public frc::Subsystem {
public:
	SwerveDriveTrain();
	void InitDefaultCommand() override;
	void Swerve(double speed, double twist, double rotAngle);
	std::vector<double> AdjustForSpeed(double speed);
	std::vector<double> AdjustForTwist(double twist);
	std::vector<double> AdjustForRotation(double rotAngle, double speed, bool newValues);
	void SetDriveMotors(std::vector<double> driveSpeeds, std::vector<double> twistSpeeds);
	void SetRotationMotors(std::vector<double> rotSpeeds);
	void ZeroWheels();
	void ZeroYaw();
	double EncoderToDegrees(double units);
	double EncoderToInches(double units);
	double GetNormalAngle(double angle);
	double GetWrappedAngle(double angle);
	std::vector<double> SetVector(double speed);
	bool GetFieldOriented();
	void SetFieldOriented(bool val);
private:
	double wheelAngle;
	double wheelOffset;
	std::vector<double> driveSpeeds;
	std::vector<double> twistSpeeds;
	std::vector<double> rotSpeeds;
	bool fieldOriented;

	// PID loop variables
	double prevUnits;
	double prevTarget;
	PID *rotationPID;
	double estimatedWheelAngle;
	double multiplier;

	// Value for determining if the wheel angle reading is fresh or not.
	int oldVal;

	// Debugging values
	double lastJ;
	double lastWA;
	double correctionSum;
};


#endif /* SRC_SUBSYSTEMS_SWERVEDRIVETRAIN_H_ */
