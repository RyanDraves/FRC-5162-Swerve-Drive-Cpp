/*
 * PID.h
 *
 *  Created on: May 26, 2018
 *      Author: brrobotics5162
 */

#ifndef SRC_PID_PID_H_
#define SRC_PID_PID_H_

class PID {
public:
	PID(double p, double i, double d, double max, double min);
	void PIDWrite(double error);
	double GetCorrection();
	double GetPreviousError();
	void SetP(double p);
	void SetI(double i);
	void SetD(double d);
	void SetCorrection(double correction);
	void ResetCumulativeError();
private:
	double kP;
	double kI;
	double kD;
	double correction;
	double prevTarget;
	double prevError;
	double cumulativeError;
	double maximum;
	double minimum;
	bool useMaximum;
};

#endif /* SRC_PID_PID_H_ */
