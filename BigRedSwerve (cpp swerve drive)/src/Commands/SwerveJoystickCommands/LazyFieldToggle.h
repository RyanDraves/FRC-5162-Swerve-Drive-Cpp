/*
 * LazyFieldToggle.h
 *
 *  Created on: Mar 28, 2018
 *      Author: RyanTheRat
 */

#ifndef SRC_COMMANDS_SWERVEJOYSTICKCOMMANDS_LAZYFIELDTOGGLE_H_
#define SRC_COMMANDS_SWERVEJOYSTICKCOMMANDS_LAZYFIELDTOGGLE_H_


#include <Commands/Command.h>

class LazyFieldToggle : public frc::Command {
public:
	LazyFieldToggle();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};


#endif /* SRC_COMMANDS_SWERVEJOYSTICKCOMMANDS_LAZYFIELDTOGGLE_H_ */
