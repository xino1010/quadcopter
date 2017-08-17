#include "Quadcopter.h"

Quadcopter *quadcopter;

void fooInterrupt() {
  quadcopter->dmpDataReady();
}

void setup() {
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  #ifdef DEBUG
    Serial.begin(115200);
    while (!Serial){}
  #endif

  Serial.println("llega");

  quadcopter = new Quadcopter();
  quadcopter->enableLED();

  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), fooInterrupt, RISING);

  // Connect Motors
  quadcopter->connectMotors();
  delay(50);

  // Send a minimum signal to prepare the motors
  quadcopter->armMotors();
  delay(50);

	// Init MPU9250
	quadcopter->initIMU();

  // Calculate offsets
  quadcopter->calculateOffsets();
  
	#ifdef DEBUG
	  Serial.println("Quadcopter initialized");
  #endif

  quadcopter->disableLED();
}

void loop() {

  if (!quadcopter->getDmpReady()) {
    return;
  }

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
