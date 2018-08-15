/*
 * PID.cpp
 *
 *  Created on: May 26, 2018
 *      Author: brrobotics5162
 */

#include "PID.h"


PID::PID(double p, double i, double d, double max, double min) {
	kP = p;
	kI = i;
	kD = d;
	correction = 0;
	prevTarget = 0;
	prevError = 0;
	cumulativeError = 0;
	maximum = max;
	minimum = min;
	useMaximum = !(maximum == 0 && minimum == 0);
}

void PID::PIDWrite(double error) {
	// P
	double pCorrection = kP * error;

	// I
	cumulativeError += error;
	double iCorrection = kI * cumulativeError;

	// D
	double slope = error - prevError;
	double dCorrection = kD * slope;
	prevError = error;

	correction = pCorrection + iCorrection + dCorrection;
	if (useMaximum) {
		if (correction > maximum) {
			correction = maximum;
		}
		else if (correction < minimum) {
			correction = minimum;
		}
	}
}

double PID::GetCorrection() {
	return correction;
}

double PID::GetPreviousError() {
	return prevError;
}

void PID::SetP(double p) {
	kP = p;
}

void PID::SetI(double i) {
	kI = i;
}

void PID::SetD(double d) {
	kD = d;
}

void PID::SetCorrection(double correction) {
	this->correction = correction;
}

void PID::ResetCumulativeError() {
	cumulativeError = 0;
}
