#include "Arduino.h"
#include "PID.h"

PID::PID(double kP, double kI, double kD, double lowerLimit, double upperLimit) {
	this->kP = kP;
	this->kI = kI;
	this->kD = kD;
	this->lowerLimit = lowerLimit;
	this->upperLimit = upperLimit;
	this->_time = millis();
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
	lastDt = _time;
	_time = millis();
	double dt = (_time - lastDt) / 1000.0;
	double currentError = currentPoint - desiredPoint;
	I += currentError * dt;
	double D = kD * ((currentError - lastError) / dt);
	lastError = currentError;
	double PID = (kP * currentError) + (kI * I) + (kD * D);
	if (PID < lowerLimit)
		PID = lowerLimit;
	if (PID > upperLimit)
		PID = upperLimit;
	return PID;
}

void PID::reset() {
	I = 0;
	desiredPoint = 0;
	currentPoint = 0;
	lastDt = millis();
	lastError = 0;
}
