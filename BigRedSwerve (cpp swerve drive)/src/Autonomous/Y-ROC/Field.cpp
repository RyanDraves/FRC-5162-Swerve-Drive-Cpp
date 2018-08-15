/*
 * Field.cpp
 *
 *  Created on: May 29, 2018
 *      Author: brrobotics5162
 */

#include "Field.h"
#include "WPILib.h"

bool CrissCross(const point& location, const point& target, const line* objectLine);
bool onSegment(const point& p, const point& q, const point& r);
int orientation(const point& p, const point& q, const point& r);

// Initializer list works here, couldn't get it to work otherwise.
Field::Field() {
	/*
	 * East (non-circular) obstacle on the field is hard coded into the field
	 * object with a vector of points to make the given shape. Enter the points
	 * in a clockwise order so the lines that make up those points are correct.
	 *
	 * I have more complicated collision code I can dig up, but rather than
	 * subjecting myself to poorly annotated code I wrote, I can advise that
	 * the objects be entered with an error margin attached to their demensions.
	 *
	 * Basically, add a small margin to the entered dimension of the field object.
	 */

	// I couldn't figure out how to do this all with pointers so just
	// clear() it after each creation and hope nothing bad happens.

	std::vector<point> field;
	field.push_back(point(0,100));
	field.push_back(point(100,100));
	field.push_back(point(100,0));
	field.push_back(point(0,0));
	obstacles.push_back(new shape(field));
	field.clear();

	// Margin example: a 20x20 square on the field, entered with a 0.5 error margin.
	std::vector<point> testSquare;
	testSquare.push_back(point(39.5, 40.5));
	testSquare.push_back(point(60.5, 40.5));
	testSquare.push_back(point(60.5, 19.5));
	testSquare.push_back(point(39.5, 19.5));
	obstacles.push_back(new shape(testSquare));
	testSquare.clear();
}

bool Field::IsCollision(point location, point target) {
	for (shape *ob : obstacles) {
		for (line *line : ob->lines) {
			if (CrissCross(location, target, line)) {
				SmartDashboard::PutNumber("px", line->pointA.x);
				SmartDashboard::PutNumber("py", line->pointA.y);
				SmartDashboard::PutNumber("qx", line->pointB.x);
				SmartDashboard::PutNumber("qy", line->pointB.y);
				return true;
			}
		}
	}
	return false;
}

// A function and helper function that determine wether or not two line segments cross.
// I dug this out of my Halite II AI competition repository.
bool CrissCross(const point& location, const point& target, const line* objectLine) {
	/*
		location & target, objectLine->pointA & objectLine->pointB need to not be crossing.
		   p1		 q1				p2					q2
		Thanks to www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/ for writing a C++ example.
		Note: This does not check for a special collinear case because TargetOrPlanetInTheWay will pick that off much better.
	*/

	// Find the four orientations needed for general and
	// special cases
	int o1 = orientation(location, target, objectLine->pointA);
	int o2 = orientation(location, target, objectLine->pointB);
	int o3 = orientation(objectLine->pointA, objectLine->pointB, location);
	int o4 = orientation(objectLine->pointA, objectLine->pointB, target);

	// General case
	if (o1 != o2 && o3 != o4) {
		return true;
	}
	// Special Cases
	// p1, q1 and p2 are colinear and p2 lies on segment p1q1
	if (o1 == 0 && onSegment(location, objectLine->pointA, target)) return true;
	// p1, q1 and q2 are colinear and q2 lies on segment p1q1
	if (o2 == 0 && onSegment(location, objectLine->pointB, target)) return true;
	// p2, q2 and p1 are colinear and p1 lies on segment p2q2
	if (o3 == 0 && onSegment(objectLine->pointA, location, objectLine->pointB)) return true;
	// p2, q2 and q1 are colinear and q1 lies on segment p2q2
	if (o4 == 0 && onSegment(objectLine->pointA, target, objectLine->pointB)) return true;
	return false;
}
//Helper function also written by geeksforgeeks.
bool onSegment(const point& p, const point& q, const point& r)
{
    if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
        q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y))
       return true;

    return false;
}
//Helper function also written by geeksforgeeks.
int orientation(const point& p, const point& q, const point& r) {
	// See www.geeksforgeeks.org/orientation-3-ordered-points/
	// for details of below formula.
	int val = (q.y - p.y) * (r.x - q.x) -
		(q.x - p.x) * (r.y - q.y);

	if (val == 0) return 0;  // colinear

	return (val > 0) ? 1 : 2; // clock or counterclock wise
}
