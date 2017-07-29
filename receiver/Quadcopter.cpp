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
	radio =  RF24(NFR24L01_CE, NFR24L01_CSN);
	desiredPitch = 0;
	desiredRoll = 0;
	desiredYaw = 0;
	radio.begin();
	radio.setPALevel(RF24_PA_HIGH); // RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
	radio.openReadingPipe(1, RADIO_ADDRESS);
	radio.startListening();

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

	// Calculate the pids from the setpoints and the current point
	double resultPitch = pidPitch.calculate();
	double resultRoll = pidRoll.calculate();
	double resultYaw = pidYaw.calculate();
	
	// The velocities are obtained from the calculations of the pids
	int tmpVFL = getThrottle() + resultRoll + resultPitch + resultYaw;
	int tmpVFR = getThrottle() - resultRoll + resultPitch - resultYaw;
	int tmpVBL = getThrottle() + resultRoll - resultPitch - resultYaw;
	int tmpVBR = getThrottle() - resultRoll - resultPitch + resultYaw;

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

int Quadcopter::getVelocityFL() {
	return vFL;
}

void Quadcopter::setVelocityFL(int vFL) {
	this->vFL = vFL;
}

int Quadcopter::getVelocityFR() {
	return vFR;
}

void Quadcopter::setVelocityFR(int vFR) {
	this->vFR = vFR;
}

int Quadcopter::getVelocityBL() {
	return vBL;
}

void Quadcopter::setVelocityBL(int vBL) {
	this->vBL = vBL;
}

int Quadcopter::getVelocityBR() {
	return vBR;
}

void Quadcopter::setVelocityBR(int vBR) {
	this->vBR = vBR;
}

// RADIO
int Quadcopter::getDesiredPitch() {
	return desiredPitch;
}

void Quadcopter::setDesiredPitch(int desiredPitch) {
	this->desiredPitch = desiredPitch;
	pidPitch.setDesiredPoint(this->desiredPitch);
}

int Quadcopter::getDesidedRoll() {
	return desiredRoll;
}

void Quadcopter::setDesiredRoll(int desiredRoll) {
	this->desiredRoll = desiredRoll;
	pidRoll.setDesiredPoint(this->desiredRoll);
}

int Quadcopter::getDesiredYaw() {
	return desiredYaw;
}

void Quadcopter::setDesiredYaw(int desiredYaw) {
	this->desiredYaw = desiredYaw;
	pidYaw.setDesiredPoint(this->desiredYaw);
}

int Quadcopter::getThrottle() {
	return throttle;
}

void Quadcopter::setThrottle(int throttle) {
	this->throttle = throttle;
}

void Quadcopter::updateRadioInfo() {

	if (radio.available()) {
		SetPoints sp;
		radio.read(&sp, sizeof(SetPoints));
		setDesiredPitch(sp.pitch);
		setDesiredRoll(sp.roll);
		setDesiredYaw(sp.yaw);
		setThrottle(sp.throttle);
	}

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
int Quadcopter::getCurrentPitch() {
	return currentPitch;
}

void Quadcopter::setCurrentPitch(int currentPitch) {
	this->currentPitch = currentPitch;
}

int Quadcopter::getCurrentRoll() {
	return currentRoll;
}

void Quadcopter::setCurrentRoll(int currentRoll) {
	this->currentRoll = currentRoll;
}

int Quadcopter::getCurrentYaw() {
	return currentYaw;
}

void Quadcopter::setCurrentYaw(int currentYaw) {
	this->currentYaw = currentYaw;
}