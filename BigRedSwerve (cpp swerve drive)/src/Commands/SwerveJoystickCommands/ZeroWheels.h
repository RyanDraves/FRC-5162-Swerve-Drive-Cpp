/*
 * ZeroWheels.h
 *
 *  Created on: May 26, 2018
 *      Author: brrobotics5162
 */

#ifndef SRC_COMMANDS_SWERVEJOYSTICKCOMMANDS_ZEROWHEELS_H_
#define SRC_COMMANDS_SWERVEJOYSTICKCOMMANDS_ZEROWHEELS_H_


#include <Commands/Command.h>

class ZeroWheels : public frc::Command {
public:
	ZeroWheels();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};


#endif /* SRC_COMMANDS_SWERVEJOYSTICKCOMMANDS_ZEROWHEELS_H_ */
