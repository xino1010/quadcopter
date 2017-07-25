#include "Arduino.h"
#include "PID.h"

PID::PID(double kP, double kI, double kD, double lowerLimit, double upperLimit) {
	this->kP = kP;
	this->kI = kI;
	this->kD = kD;
	this->lowerLimit = lowerLimit;
	this->upperLimit = upperLimit;
}

void PID::setKp(double kP) {
	this->kP = kP;
}

void PID::setKi(double kI) {
	this->kI = kI;
}

void PID::setKd(double kD) {
	this->kD = kD;
}

void PID::setDesiredPoint(double desiredPoint) {
	this->desiredPoint = desiredPoint;
}

void PID::setCurrentPoint(double currentPoint) {
	this->currentPoint = currentPoint;
}

double PID::calculate() {
	double dt = (millis() - lastDt) / 1000.0;
	lastDt = millis();
	double currentError = currentPoint - desiredPoint;
	P = kP * currentError;
	I = I + (kI * currentError);
	D = kD * ((currentError - lastError) / dt);
	lastError = currentError;
	double PID = P + I + D;
	if (PID < lowerLimit)
		PID = lowerLimit;
	if (PID > upperLimit)
		PID = upperLimit;
	return PID;
}
