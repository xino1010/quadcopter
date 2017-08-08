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

	// Init MPU9250
	quadcopter->initIMU();

  // Check if MPU9250 is calibrated
  if (!quadcopter->isCalibrated()) {
    quadcopter->calibrateIMU();
    if (!quadcopter->isCalibrated()) {
      #ifdef DEBUG
        Serial.println("MPU9250 can not be calibrated...");
      #endif
      while(true) {
        delay(50);
      }
    }
    else {
    #ifdef DEBUG
      Serial.println("MPU9250 has been calibrated on the second attempt");
    #endif
    }
  }
  else {
    #ifdef DEBUG
      Serial.println("MPU9250 is calibrated");
    #endif
  }

	// Connect Motors
	//quadcopter->connectMotors();
	delay(50);

	// Send a minimum signal to prepare the motors
	//quadcopter->armMotors();
	delay(50);

	#ifdef DEBUG
	  Serial.println("Quadcopter initialized");
  #endif
}

void loop() {
	// Read data from radio
	quadcopter->updateRadioInfo();

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

	// Wait a moment...
	delay(50);
}
