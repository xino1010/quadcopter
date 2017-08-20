#include "Arduino.h"
#include "PID.h"

PID::PID(float kP, float kI, float kD, double lowerLimit, double upperLimit) {
  this->kP = kP;
  this->kI = kI;
  this->kD = kD;
  this->lowerLimit = lowerLimit;
  this->upperLimit = upperLimit;
  this->_time = millis();
}

void PID::setKp(float kP) {
  this->kP = kP;
}

float PID::getKp() {
  return kP;
}

void PID::setKi(float kI) {
  this->kI = kI;
}

float PID::getKi() {
  return kI;
}

void PID::setKd(float kD) {
  this->kD = kD;
}

float PID::getKd() {
  return kD;
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
  double D = (currentError - lastError) / dt;
  lastError = currentError;
  double PID = (kP * currentError) + (kI * I) + (kD * D);
  return constrain(PID, lowerLimit, upperLimit);
}

void PID::reset() {
  I = 0;
  desiredPoint = 0;
  currentPoint = 0;
  _time = millis();
  lastDt = millis();
  lastError = 0;
}
