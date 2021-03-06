#include <Arduino.h>

#include "Quadcopter.h"

Quadcopter *quadcopter;

void setup() {
  Wire.begin();

  #ifdef DEBUG
    Serial.begin(38400);
    while (!Serial){}
    delay(1000);
  #endif

  quadcopter = new Quadcopter();
  quadcopter->enableLED();

  // Connect Motors
  quadcopter->connectMotors();
  delay(50);

  // Send a minimum signal to prepare the motors
  quadcopter->armMotors();
  delay(50);

	// Init MPU9250
	quadcopter->initIMU();

  // Calculate offsets in order to get correct angles
  quadcopter->calculateIMUOffsets();

	#ifdef DEBUG
	  Serial.println("Quadcopter initialized");
  #endif

  quadcopter->disableLED();
}

void loop() {

  #ifdef NORMAL_MODE
  	// Read data from radio
  	quadcopter->updateRadioInfo();
  #endif

  #ifdef CALIBRATION_MODE
    // Read constants of kpi from radio
    quadcopter->updatePIDInfo();
  #endif

  if (quadcopter->getControlMode() == CONTROL_MODE_OFF) {
    // Set minim velocity to all motors
    quadcopter->stopMotors();
  }
  else {
    // Read angles from sensor
    quadcopter->updateAngles();

    // Calculate velocities of each motor depending of ControlMode
    quadcopter->calculateVelocities();
  }

  // Send velocities to motors
  quadcopter->updateMotorsVelocities();
}
