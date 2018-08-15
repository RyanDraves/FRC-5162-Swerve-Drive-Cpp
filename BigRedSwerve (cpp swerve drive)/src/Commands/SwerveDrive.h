/*
 * SwerveDrive.h
 *
 *  Created on: Mar 28, 2018
 *      Author: RyanTheRat
 */

#ifndef SRC_COMMANDS_SWERVEDRIVE_H_
#define SRC_COMMANDS_SWERVEDRIVE_H_


#include <Commands/Command.h>
#include "../OI.h"
//#include "../Robot.h" should be here (see source file for details).


#include "../Autonomous/Y-ROC/AutoBot.h"
#include "../Autonomous/Y-ROC/Field.h"
#include "../Autonomous/Y-ROC/Misc.h"

class SwerveDrive : public frc::Command {
private:
	Autobot testBot;
public:
	SwerveDrive();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};


#endif /* SRC_COMMANDS_SWERVEDRIVE_H_ */
