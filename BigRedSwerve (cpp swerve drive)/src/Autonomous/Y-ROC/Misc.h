/*
 * Misc.h
 *
 *  Created on: May 29, 2018
 *      Author: brrobotics5162
 */

#ifndef SRC_AUTONOMOUS_Y_ROC_MISC_H_
#define SRC_AUTONOMOUS_Y_ROC_MISC_H_

#include <vector>

struct point {
	point(double x, double y) : x(x), y(y) {}
	double x;
	double y;
	bool equals(point check);
};

struct line {
	line(point a, point b) : pointA(a), pointB(b) {}
	point pointA;
	point pointB;
};

struct shape {
	shape(std::vector<point> points);
	std::vector<line*> lines;
};


#endif /* SRC_AUTONOMOUS_Y_ROC_MISC_H_ */
