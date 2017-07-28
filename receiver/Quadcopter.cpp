#include "Quadcopter.h"

Quadcopter::Quadcopter() {
  	// PID
  	pidPitch = PID(kpPitch, kiPitch, kdPitch, MIN_VALUE_PID, MAX_VALUE_PID);
  	pidRoll = PID(kpRoll, kiRoll, kdRoll, MIN_VALUE_PID, MAX_VALUE_PID);
  	pidYaw = PID(kpYaw, kiYaw, kdYaw, MIN_VALUE_PID, MAX_VALUE_PID);
	
	// MOTORS
	vFL = 0;
	vFR = 0;
	vBL = 0;
	vBR = 0;

	// RADIO
	desiredPitch = 0;
	desiredRoll = 0;
	desiredYaw = 0;

	// IMU
	currentPitch = 0;
	currentRoll = 0;
	currentYaw = 0;
}

// BMP180
float Quadcopter::getAltitude() {
	return bmp.readAltitude();
}

float Quadcopter::getTemperature() {
	return bmp.readTemperature();
}

// MOTORS
void Quadcopter::connectMotors() {
	motorFL.attach(PIN_MOTOR_FL);
	motorFR.attach(PIN_MOTOR_FR);
	motorBL.attach(PIN_MOTOR_BL);
	motorBR.attach(PIN_MOTOR_BR);
}

void Quadcopter::armMotors() {
	motorFL.writeMicroseconds(ARM_MOTOR);
	motorFR.writeMicroseconds(ARM_MOTOR);
	motorBL.writeMicroseconds(ARM_MOTOR);
	motorBR.writeMicroseconds(ARM_MOTOR);
}

void Quadcopter::calculateVelocities() {
	double resultPitch = pidPitch.calculate();
	double resultRoll = pidRoll.calculate();
	double resultYaw = pidYaw.calculate();
	
	// The velocities are obtained from the calculations of the pids
	float tmpVFL = getThrottle() + resultRoll + resultPitch + resultYaw;
	float tmpVFR = getThrottle() - resultRoll + resultPitch - resultYaw;
	float tmpVBL = getThrottle() + resultRoll - resultPitch - resultYaw;
	float tmpVBR = getThrottle() - resultRoll - resultPitch + resultYaw;

	// Check that the speeds do not exceed the limits
	setVelocityFL(constrain(tmpVFL, MIN_VALUE_MOTOR, MAX_VALUE_MOTOR));
	setVelocityFR(constrain(tmpVFR, MIN_VALUE_MOTOR, MAX_VALUE_MOTOR));
	setVelocityBL(constrain(tmpVBL, MIN_VALUE_MOTOR, MAX_VALUE_MOTOR));
	setVelocityBR(constrain(tmpVBR, MIN_VALUE_MOTOR, MAX_VALUE_MOTOR));

	#ifdef DEBUG
		Serial.print("FL: ");
		Serial.print(getVelocityFL());
		Serial.print("\tFR: ");
		Serial.print(getVelocityFR());
		Serial.print("\tBL: ");
		Serial.print(getVelocityBL());
		Serial.print("\tBR: ");
		Serial.println(getVelocityBR());
	#endif
}

void Quadcopter::updateMotorsVelocities() {
	motorFL.writeMicroseconds(getVelocityFL());
	motorFR.writeMicroseconds(getVelocityFR());
	motorBL.writeMicroseconds(getVelocityBL());
	motorBR.writeMicroseconds(getVelocityBR());
}

float Quadcopter::getVelocityFL() {
	return vFL;
}

void Quadcopter::setVelocityFL(float vFL) {
	this->vFL = vFL;
}

float Quadcopter::getVelocityFR() {
	return vFR;
}

void Quadcopter::setVelocityFR(float vFR) {
	this->vFR = vFR;
}

float Quadcopter::getVelocityBL() {
	return vBL;
}

void Quadcopter::setVelocityBL(float vBL) {
	this->vBL = vBL;
}

float Quadcopter::getVelocityBR() {
	return vBR;
}

void Quadcopter::setVelocityBR(float vBR) {
	this->vBR = vBR;
}

// RADIO
float Quadcopter::getDesiredPitch() {
	return desiredPitch;
}

void Quadcopter::setDesiredPitch(float desiredPitch) {
	this->desiredPitch = desiredPitch;
	pidPitch.setDesiredPoint(this->desiredPitch);
}

float Quadcopter::getDesidedRoll() {
	return desiredRoll;
}

void Quadcopter::setDesiredRoll(float desiredRoll) {
	this->desiredRoll = desiredRoll;
	pidRoll.setDesiredPoint(this->desiredRoll);
}

float Quadcopter::getDesiredYaw() {
	return desiredYaw;
}

void Quadcopter::setDesiredYaw(float desiredYaw) {
	this->desiredYaw = desiredYaw;
	pidYaw.setDesiredPoint(this->desiredYaw);
}

float Quadcopter::getThrottle() {
	return throttle;
}

void Quadcopter::setThrottle(float throttle) {
	this->throttle = throttle;
}

void Quadcopter::updateRadioInfo() {
	JoystickInfo ji;
	setDesiredPitch(ji.pitch);
	setDesiredRoll(ji.roll);
	setDesiredYaw(ji.yaw);
	setThrottle(ji.throttle);

	#ifdef DEBUG
		Serial.print("THROTTLE: ");
		Serial.print(getThrottle());
		Serial.print("\tPITCH: ");
		Serial.print(getDesiredPitch());
		Serial.print("\tROLL: ");
		Serial.print(getDesidedRoll());
		Serial.print("\tYAW: ");
		Serial.println(getDesiredYaw());
	#endif
}

// IMU
float Quadcopter::getCurrentPitch() {
	return currentPitch;
}

void Quadcopter::setCurrentPitch(float currentPitch) {
	this->currentPitch = currentPitch;
}

float Quadcopter::getCurrentRoll() {
	return currentRoll;
}

void Quadcopter::setCurrentRoll(float currentRoll) {
	this->currentRoll = currentRoll;
}

float Quadcopter::getCurrentYaw() {
	return currentYaw;
}

void Quadcopter::setCurrentYaw(float currentYaw) {
	this->currentYaw = currentYaw;
}