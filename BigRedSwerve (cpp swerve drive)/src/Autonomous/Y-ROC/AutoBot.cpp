/*
 * AutoBot.cpp
 *
 *  Created on: May 29, 2018
 *      Author: brrobotics5162
 */

#define ROBOT_SPEED 6.874 // Feet per second per max motor usage.
						  // This is estimated with 0.1 speed's FPS * 10.

#include "AutoBot.h"
#include <Math.h>
#include "WPILib.h"

double orient_towards_in_rad(const point& location, const point& target);

Autobot::Autobot(double startX, double startY, double endX, double endY) : location(startX, startY), target(endX, endY), tempTarget(endX, endY), field() {

}

void Autobot::SetLocation(double x, double y) {
	location.x = x;
	location.y = y;
}

void Autobot::UpdateLocation(double angle, double motorSpeed) {
	// This is a conversion from a wheel angle with 0 degrees due north
	// to 0 degrees due east for the sake of mathematics.
	angle = fmod(angle + 90, 360);

	// This should be replaced by encoder units on the drive motor later.
	// The motor will be annoying with its feedback and claim it's currently at
	// 0.4% speed when it's at rest. Ugh.
	if (fabs(motorSpeed) > 0.01) {
		// 				(20 milliseconds because that's the length of each code loop)
		//					radian conversion	   motorSpeed   robot FPS    seconds
		location.x += cos((angle * M_PI / 180.0)) * motorSpeed * ROBOT_SPEED * 0.020;
		location.y += sin((angle * M_PI / 180.0)) * motorSpeed * ROBOT_SPEED * 0.020;
	}
	SmartDashboard::PutNumber("wheel", angle);
	SmartDashboard::PutNumber("location_x", location.x);
	SmartDashboard::PutNumber("location_y", location.y);
	SmartDashboard::PutNumber("tempTarget_x", tempTarget.x);
	SmartDashboard::PutNumber("tempTarget_y", tempTarget.y);
	SmartDashboard::PutNumber("Speed", motorSpeed);
}

double Autobot::GetNextTwist() {
	return 0;
}

double Autobot::GetNextAngle() {
	if (!tempTarget.equals(target) || field.IsCollision(location, tempTarget)) {
		SetNewTarget();
	}
	double nextAngle = 180.0 / M_PI * orient_towards_in_rad(location, tempTarget);
	/*
	 * Now we have to convert from mathematical degrees to what the joystick uses.
	 *										90							0
	 *									180	   0					-90	  90
	 *										270						  +/-180
	*/
	nextAngle -= 90; // Reorientate.
	nextAngle -= 2 * nextAngle; // Invert.
	nextAngle = fmod(nextAngle + 540, 360) - 180; // Scale back to -180 to 180.
	return nextAngle;
}

void Autobot::SetNewTarget() {
	// Set the temp target to the final target if there's no collision.
	if (!field.IsCollision(location, target)) {
		tempTarget = target;
	}
	// Otherwise find the nearest
	else {
		tempTarget = {location.x + sin(orient_towards_in_rad(location, target)), location.y + cos(orient_towards_in_rad(location, target))};
		// Find the nearest target with 90 degrees that doesn't have a collision.
		int maxCorrections = 181;
		bool left = true;
		while (field.IsCollision(location, tempTarget)) {
			maxCorrections--;
			const double angle_rad = orient_towards_in_rad(location, tempTarget);
			double new_target_dx;
			double new_target_dy;
			//Alternate checking 1 degree left/right.
			if (left) {
				new_target_dx = cos(angle_rad + (M_PI / 180.0)*(181.0 - maxCorrections));
				new_target_dy = sin(angle_rad + (M_PI / 180.0)*(181.0 - maxCorrections));
				left = false;
			}
			else {
				new_target_dx = cos(angle_rad - (M_PI / 180.0)*(181.0 - maxCorrections));
				new_target_dy = sin(angle_rad - (M_PI / 180.0)*(181.0 - maxCorrections));
				left = true;
			}
			tempTarget = { location.x + new_target_dx, location.y + new_target_dy };
		}
		// Don't move if a correction wasn't found.
		if (!maxCorrections) {
			tempTarget = location;
			target = location;
		}
	}
}

bool Autobot::IsFinished() {
	if (fabs(location.x - target.x) < 1 && fabs(location.y - target.y) < 1) {
		return true;
	}
	return false;
}

double orient_towards_in_rad(const point& location, const point& target) {
	const double dx = target.x - location.x;
	const double dy = target.y - location.y;

	// + 2 * M_PI
	return atan2(dy, dx);
}
