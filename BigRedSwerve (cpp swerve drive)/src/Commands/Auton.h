/*
 * Auton.h
 *
 *  Created on: May 26, 2018
 *      Author: brrobotics5162
 */

#ifndef SRC_COMMANDS_AUTON_H_
#define SRC_COMMANDS_AUTON_H_


#include <Commands/Command.h>

#include "../Autonomous/Y-ROC/AutoBot.h"
#include "../Autonomous/Y-ROC/Field.h"
#include "../Autonomous/Y-ROC/Misc.h"

class Auton : public frc::Command {
private:
	Autobot robot;
public:
	Auton(double startX, double startY, double endX, double endY);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};



#endif /* SRC_COMMANDS_AUTON_H_ */
