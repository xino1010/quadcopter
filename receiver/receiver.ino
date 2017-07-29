#include "Quadcopter.h"

Quadcopter quadcopter;

void setup() {
	// Connect Motors
	quadcopter.connectMotors();
	delay(50);
	// Send a minimum signal to prepare the motors
	quadcopter.armMotors();
	delay(50);
}

void loop() {

	// Read data from radio
	quadcopter.updateRadioInfo();

	// Calculate velocities of each motor
	quadcopter.calculateVelocities();

	// Send velocities to motors
	quadcopter.updateMotorsVelocities();

	delay(50);
}
