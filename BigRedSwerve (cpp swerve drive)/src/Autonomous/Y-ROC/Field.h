/*
 * Field.h
 *
 *  Created on: May 29, 2018
 *      Author: brrobotics5162
 */

#ifndef SRC_AUTONOMOUS_Y_ROC_FIELD_H_
#define SRC_AUTONOMOUS_Y_ROC_FIELD_H_

#include <vector>
#include "Misc.h"

class Field {
public:
	Field();
	bool IsCollision(point location, point target);
private:
	std::vector<shape*> obstacles;
};


#endif /* SRC_AUTONOMOUS_Y_ROC_FIELD_H_ */
