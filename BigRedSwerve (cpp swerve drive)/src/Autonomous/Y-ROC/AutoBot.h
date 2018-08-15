/*
 * AutoBot.h
 *
 *  Created on: May 29, 2018
 *      Author: brrobotics5162
 */

#ifndef SRC_AUTONOMOUS_Y_ROC_AUTOBOT_H_
#define SRC_AUTONOMOUS_Y_ROC_AUTOBOT_H_

#include "Field.h"
#include "Misc.h"

class Autobot {
public:
	Autobot(double startX, double startY, double endX, double endY);
	void SetLocation(double x, double y);
	void UpdateLocation(double angle, double motorSpeed);
	double GetNextTwist();
	double GetNextAngle();
	bool IsFinished();
private:
	void SetNewTarget();
	point location;
	point target;
	point tempTarget;
	Field field;
};


#endif /* SRC_AUTONOMOUS_Y_ROC_AUTOBOT_H_ */
