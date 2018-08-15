#include "RobotMap.h"
#include "WPILib.h"
#include "ctre/Phoenix.h"

std::shared_ptr<AHRS> RobotMap::ahrs;

std::shared_ptr<VictorSP> RobotMap::driveMotor0;
std::shared_ptr<VictorSP> RobotMap::driveMotor1;
std::shared_ptr<VictorSP> RobotMap::driveMotor2;
std::shared_ptr<VictorSP> RobotMap::driveMotor3;

std::shared_ptr<WPI_TalonSRX> RobotMap::rotationMotor0;
std::shared_ptr<WPI_TalonSRX> RobotMap::rotationMotor1;
std::shared_ptr<WPI_TalonSRX> RobotMap::rotationMotor2;
std::shared_ptr<WPI_TalonSRX> RobotMap::rotationMotor3;

std::shared_ptr<DifferentialDrive> RobotMap::robot_drive;

void RobotMap::init() {
	LiveWindow::GetInstance()->DisableAllTelemetry();

	ahrs.reset(new AHRS(frc::SPI::Port::kMXP));

	driveMotor0.reset(new VictorSP(0));
	// PWM ports for Victor's, so they're on 0, 1, and 2.
	driveMotor1.reset(new VictorSP(0));
	driveMotor2.reset(new VictorSP(1));
	driveMotor3.reset(new VictorSP(2));

	rotationMotor0.reset(new WPI_TalonSRX(1));
	rotationMotor1.reset(new WPI_TalonSRX(2));
	rotationMotor2.reset(new WPI_TalonSRX(3));
	rotationMotor3.reset(new WPI_TalonSRX(4));
}
