/*
 * Misc.cpp
 *
 *  Created on: May 29, 2018
 *      Author: brrobotics5162
 */

#include "Misc.h"

bool point::equals(point check) {
	return x == check.x && y == check.y;
}

shape::shape(std::vector<point> points) {
	for (unsigned int i = 0; i < points.size(); i++) {
		line *newLine = new line(points[i], points[(i + 1) % points.size()]);
		lines.push_back(newLine);
	}
}
