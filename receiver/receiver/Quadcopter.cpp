#include "Quadcopter.h"

Quadcopter::Quadcopter() {

  // LED
  pinMode(LED_PIN , OUTPUT);  //definir pin como salida

  // BMP180
  if (!bmp.begin()) {
    #ifdef DEBUG_BMP
      Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    #endif
    while (true) {
      delay(1000);
    }
  }
  delay(1000);
  offsetAltitude = getAltitude();
  previousAltitudeRead = millis();

	// PID
	pidPitch = new PID(kpPitch, kiPitch, kdPitch, MIN_PITCH, MAX_PITCH);
	pidRoll = new PID(kpRoll, kiRoll, kdRoll, MIN_ROLL, MAX_ROLL);
	pidYaw = new PID(kpYaw, kiYaw, kdYaw, MIN_YAW, MAX_YAW);
  pidDistance = new PID(kpDistance, kiDistance, kdDistance, MIN_DISTANCE, MAX_DISTANCE);
  pidDistance->setDesiredPoint(DESIRED_DISTANCE);
  pidAltitude = new PID(kpAltitude, kiAltitude, kdAltitude, MIN_ALTITUDE, MAX_ALTITUDE);
  pidAltitude->setDesiredPoint(DESIRED_ALTITUDE);
  desiredPitch = 0;
  lastDesiredPitch = 0;
  pidPitch->setDesiredPoint(desiredPitch);
	desiredRoll = 0;
  lastDesiredRoll = 0;
  pidRoll->setDesiredPoint(desiredRoll);
	desiredYaw = 0;
  lastDesiredYaw = 0;
  pidYaw->setDesiredPoint(desiredYaw);
  pidDistance->setDesiredPoint(0);
  pidAltitude->setDesiredPoint(offsetAltitude);

	// MOTORS
	vFL = 0;
	vFR = 0;
	vBL = 0;
	vBR = 0;

	// RADIO
	radio = new RF24(NFR24L01_CE, NFR24L01_CSN);
  
  #ifdef NORMAL_MODE
	  throttle = ZERO_VALUE_MOTOR;
    cm = CONTROL_MODE_OFF;
  #endif
  
  #ifdef CALIBRATION_MODE
    throttle = 1150;
    cm = CONTROL_MODE_ACRO;
  #endif
  
  controlModeChange = false;
  
	radio->begin();
  radio->setDataRate(RF24_250KBPS);
	radio->setPALevel(RF24_PA_MAX); // RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
	radio->openReadingPipe(1, radioAddress);
	radio->startListening();

	// IMU
	currentPitch = 0;
	currentRoll = 0;
	currentYaw = 0;
	temperatureIMU = 0;
  offsetPitch = 0;
  offsetRoll = 0;
  offsetYaw = 0;

	// HC-SR04
 	//pinMode(HCSR04_ECHO_PIN, INPUT);
 	//pinMode(HCSR04_TRIGGER_PIN, OUTPUT);

}

int Quadcopter::myAbs(int value) {
  return value > 0 ? value : value * -1;
}

// BMP180
float Quadcopter::getAltitude() {
	float altitude = bmp.readAltitude();
	#ifdef DEBUG_BMP
		Serial.print("BMP180 Altitude: ");
		Serial.print(altitude);
		Serial.println("m");
	#endif
	return altitude;
}

float Quadcopter::getTemperature() {
	float temperature = bmp.readTemperature();;
	#ifdef DEBUG_BMP
		Serial.print("BMP180 Temperature: ");
		Serial.print(temperature);
		Serial.println("ºC");
	#endif
	return temperature;
}

// MOTORS
void Quadcopter::connectMotors() {
	motorFL.attach(PIN_MOTOR_FL);
	motorFR.attach(PIN_MOTOR_FR);
	motorBL.attach(PIN_MOTOR_BL);
	motorBR.attach(PIN_MOTOR_BR);

  #ifdef DEBUG_MOTORS
    Serial.println("Motors attached");
  #endif
}

void Quadcopter::armMotors() {
	motorFL.writeMicroseconds(ARM_MOTOR);
	motorFR.writeMicroseconds(ARM_MOTOR);
	motorBL.writeMicroseconds(ARM_MOTOR);
	motorBR.writeMicroseconds(ARM_MOTOR);

  #ifdef DEBUG_MOTORS
    Serial.println("Motors armed");
  #endif
}

void Quadcopter::calculateVelocities() {

  // Update current points
  pidPitch->setCurrentPoint(getCurrentPitch());
  pidRoll->setCurrentPoint(getCurrentRoll());
  pidYaw->setCurrentPoint(getCurrentYaw());

	// Calculate the pids from the setpoints and the current point
	double resultPitch = pidPitch->calculate();
	double resultRoll = pidRoll->calculate();
	double resultYaw = pidYaw->calculate();

  resultPitch = map(resultPitch, MIN_PITCH, MAX_PITCH, MIN_VALUE_PID, MAX_VALUE_PID);
  resultRoll = map(resultRoll, MIN_ROLL, MAX_ROLL, MIN_VALUE_PID, MAX_VALUE_PID);
  resultYaw = map(resultYaw, MIN_YAW, MAX_YAW, MIN_VALUE_PID, MAX_VALUE_PID);

	// The velocities are obtained from the calculations of the pids

  int tmpVFL = 0;
  int tmpVFR = 0;
  int tmpVBL = 0;
  int tmpVBR = 0;

  if (getControlMode() == CONTROL_MODE_ACRO) {
    tmpVFL = getThrottle() - resultRoll + resultPitch + resultYaw;
  	tmpVFR = getThrottle() + resultRoll + resultPitch - resultYaw;
  	tmpVBL = getThrottle() - resultRoll - resultPitch - resultYaw;
  	tmpVBR = getThrottle() + resultRoll - resultPitch + resultYaw;
  }
  else if (getControlMode() == CONTROL_MODE_HOLD_DISTANCE) {
    int dist = getDistance();
    pidDistance->setCurrentPoint(dist);
    double resultDistance = pidDistance->calculate();
    resultDistance = map(resultDistance, MIN_DISTANCE, MAX_DISTANCE, MIN_VALUE_PID, MAX_VALUE_PID);
    tmpVFL = MIN_VALUE_MOTOR - resultDistance - resultRoll + resultPitch + resultYaw;
  	tmpVFR = MIN_VALUE_MOTOR - resultDistance + resultRoll + resultPitch - resultYaw;
  	tmpVBL = MIN_VALUE_MOTOR - resultDistance - resultRoll - resultPitch - resultYaw;
  	tmpVBR = MIN_VALUE_MOTOR - resultDistance + resultRoll - resultPitch + resultYaw;
  }
  else if (getControlMode() == CONTROL_MODE_HOLD_ALTITUDE) {
    int alt = getAltitude();
    double currentPointAltitude = alt - offsetAltitude;
    pidAltitude->setCurrentPoint(currentPointAltitude);
    double resultAltitude = pidAltitude->calculate();
    resultAltitude = map(resultAltitude, MIN_ALTITUDE, MAX_ALTITUDE, MIN_VALUE_PID, MAX_VALUE_PID);
    tmpVFL = MIN_VALUE_MOTOR - resultAltitude - resultRoll + resultPitch + resultYaw;
  	tmpVFR = MIN_VALUE_MOTOR - resultAltitude + resultRoll + resultPitch - resultYaw;
  	tmpVBL = MIN_VALUE_MOTOR - resultAltitude - resultRoll - resultPitch - resultYaw;
  	tmpVBR = MIN_VALUE_MOTOR - resultAltitude + resultRoll - resultPitch + resultYaw;
  }
  controlModeChange = false;

	// Check that the speeds do not exceed the limits
	setVelocityFL(constrain(tmpVFL, MIN_VALUE_MOTOR, MAX_VALUE_MOTOR));
	setVelocityFR(constrain(tmpVFR, MIN_VALUE_MOTOR, MAX_VALUE_MOTOR));
	setVelocityBL(constrain(tmpVBL, MIN_VALUE_MOTOR, MAX_VALUE_MOTOR));
	setVelocityBR(constrain(tmpVBR, MIN_VALUE_MOTOR, MAX_VALUE_MOTOR));
}

void Quadcopter::updateMotorsVelocities() {
  #ifdef DEBUG_MOTORS
    Serial.print("CM: ");
    Serial.print(cm);
    Serial.print(" FL: ");
    Serial.print(getVelocityFL());
    Serial.print(" FR: ");
    Serial.print(getVelocityFR());
    Serial.print(" BL: ");
    Serial.print(getVelocityBL());
    Serial.print(" BR: ");
    Serial.println(getVelocityBR());
  #endif
  
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

void Quadcopter::stopMotors() {
  this->vFL = ZERO_VALUE_MOTOR;
  this->vFR = ZERO_VALUE_MOTOR;
  this->vBL = ZERO_VALUE_MOTOR;
  this->vBR = ZERO_VALUE_MOTOR;

  // Update altitude because quadcopter could have changed the position
  if (millis() - previousAltitudeRead > TIME_READ_ALTITUDE) {
    offsetAltitude = getAltitude();
    previousAltitudeRead = millis();
  }
}

// RADIO
int Quadcopter::getDesiredPitch() {
	return desiredPitch;
}

void Quadcopter::setDesiredPitch(float desiredPitch) {
	this->desiredPitch = desiredPitch;
	pidPitch->setDesiredPoint(this->desiredPitch);
}

int Quadcopter::getDesiredRoll() {
	return desiredRoll;
}

void Quadcopter::setDesiredRoll(float desiredRoll) {
	this->desiredRoll = desiredRoll;
	pidRoll->setDesiredPoint(this->desiredRoll);
}

int Quadcopter::getDesiredYaw() {
	return desiredYaw;
}

void Quadcopter::setDesiredYaw(float desiredYaw) {
	this->desiredYaw = desiredYaw;
	pidYaw->setDesiredPoint(this->desiredYaw);
}

int Quadcopter::getThrottle() {
	return throttle;
}

void Quadcopter::setThrottle(int throttle) {
	this->throttle = throttle;
}

void Quadcopter::updateRadioInfo() {

	if (radio->available()) {
    enableLED();
	  radio->read(radioData, sizeRadioData);
    
		setThrottle(radioData[0]);

    // Check if there have sent any abnormal data
    if (myAbs(lastDesiredPitch - getDesiredPitch()) > MAX_CHANGE_PITCH) {
      radioData[1] = lastDesiredPitch;
    }
    if (myAbs(lastDesiredRoll - getDesiredRoll()) > MAX_CHANGE_ROLL) {
      radioData[2] = lastDesiredRoll;
    }
    if (myAbs(lastDesiredYaw - getDesiredYaw()) > MAX_CHANGE_YAW) {
      radioData[3] = lastDesiredYaw;
    }

    // Update desired angles
		setDesiredPitch(radioData[1]);
		setDesiredRoll(radioData[2]);
		setDesiredYaw(radioData[3]);

    // Update last desired angle
    lastDesiredPitch = radioData[1];
    lastDesiredRoll = radioData[2];
    lastDesiredYaw = radioData[3];

    // Power off motors
    if (radioData[4] == LOW) {
      setControlMode(CONTROL_MODE_OFF);
    }
    // Stabilize dron and keep the distance
    else if (radioData[5] == HIGH) {
      float distance = getDistance();
      if (distance >= MIN_DISTANCE && distance < MAX_DISTANCE) {
        controlModeChange = getControlMode() != CONTROL_MODE_HOLD_DISTANCE;
        setControlMode(CONTROL_MODE_HOLD_DISTANCE);
      }
      else {
        controlModeChange = getControlMode() != CONTROL_MODE_ACRO;
        setControlMode(CONTROL_MODE_ACRO);
      }
    }
    // Stabilize dron and keep the altitud
    else if (radioData[6] == HIGH) {
      float altitudeSea = getAltitude();
      float currentAltitude = altitudeSea - offsetAltitude;
      if (currentAltitude >= MIN_ALTITUDE && currentAltitude < MAX_ALTITUDE) {
        controlModeChange = getControlMode() != CONTROL_MODE_HOLD_ALTITUDE;
        setControlMode(CONTROL_MODE_HOLD_ALTITUDE);
      }
      else {
        controlModeChange = getControlMode() != CONTROL_MODE_ACRO;
        setControlMode(CONTROL_MODE_ACRO);
      }
    }
    // Stabilize dron and respond transmitter orders
    else {
      controlModeChange = getControlMode() != CONTROL_MODE_ACRO;
      setControlMode(CONTROL_MODE_ACRO);
    }
    disableLED();
	}
}

void Quadcopter::setControlMode(int cm) {
  this->cm = cm;
}

int Quadcopter::getControlMode() {
  return cm;
}

void Quadcopter::updatePIDInfo() {
  if (radio->available()) {
    enableLED();
    radio->read(radioPIDdata, sizeRadioPIDdata);
    int resetPid = (int) radioPIDdata[3];

    if (resetPid) {
      setControlMode(CONTROL_MODE_OFF);
    }
    else {
      setControlMode(CONTROL_MODE_ACRO);
    }
    
    #ifdef CALIBRATION_PITCH
      pidPitch->setKp(radioPIDdata[0]);
      pidPitch->setKi(radioPIDdata[1]);
      pidPitch->setKd(radioPIDdata[2]);
      if (resetPid == HIGH) {
        pidPitch->reset();
      }
      #ifdef DEBUG_PID
        Serial.print("kP: ");
        Serial.print(pidPitch->getKp());
        Serial.print("\tkI: ");
        Serial.print(pidPitch->getKi());
        Serial.print("\tkD: ");
        Serial.print(pidPitch->getKd());
        Serial.print("\tReset: ");
        Serial.println(resetPid);
      #endif
    #endif

    #ifdef CALIBRATION_ROLL
      pidRoll->setKp(radioPIDdata[0]);
      pidRoll->setKi(radioPIDdata[1]);
      pidRoll->setKd(radioPIDdata[2]);
      if (resetPid == HIGH) {
        pidRoll->reset();
      }
      #ifdef DEBUG_PID
        Serial.print("kP: ");
        Serial.print(pidRoll->getKp());
        Serial.print("\tkI: ");
        Serial.print(pidRoll->getKi());
        Serial.print("\tkD: ");
        Serial.println(pidRoll->getKd());
      #endif
    #endif

    #ifdef CALIBRATION_YAW
      pidYaw->setKp(radioPIDdata[0]);
      pidYaw->setKi(radioPIDdata[1]);
      pidYaw->setKd(radioPIDdata[2]);
      if (resetPid == HIGH) {
        pidYaw->reset();
      }
      #ifdef DEBUG_PID
        Serial.print("kP: ");
        Serial.print(pidYaw->getKp());
        Serial.print("\tkI: ");
        Serial.print(pidYaw->getKi());
        Serial.print("\tkD: ");
        Serial.println(pidYaw->getKd());
      #endif
    #endif
    
    disableLED();
  }
  else {
    #ifdef DEBUG_RADIO
      Serial.println("There is no radio pid data");
    #endif
  }
}

// IMU
int Quadcopter::getCurrentPitch() {
	return currentPitch;
}

void Quadcopter::setCurrentPitch(float currentPitch) {
	this->currentPitch = currentPitch;
}

int Quadcopter::getCurrentRoll() {
	return currentRoll;
}

void Quadcopter::setCurrentRoll(float currentRoll) {
	this->currentRoll = currentRoll;
}

int Quadcopter::getCurrentYaw() {
	return currentYaw;
}

void Quadcopter::setCurrentYaw(float currentYaw) {
	this->currentYaw = currentYaw;
}

void Quadcopter::initIMU() {
  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  #ifdef DEBUG_IMU
    Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
    // VIEJO
    //Serial.print(" I should be "); Serial.println(0x71, HEX);
    // NUEVO
    Serial.print(" I should be "); Serial.println(0x73, HEX);
  #endif

  // WHO_AM_I should always be 0x68
  // VIEJO
  //if (c == 0x71) {
  // NUEVO
  if (c == 0x73) {
    #ifdef DEBUG_IMU
      Serial.println("MPU9250 is online...");
    #endif
    myIMU.initMPU9250();
    #ifdef DEBUG_IMU
      Serial.println("MPU9250 initialized for active data mode....");
    #endif

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    #ifdef DEBUG_IMU
      Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX);
      Serial.print(" I should be "); Serial.println(0x48, HEX);
    #endif

    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.magCalibration);
    #ifdef DEBUG_IMU
      // Initialize device for active mode read of magnetometer
      Serial.println("AK8963 initialized for active data mode....");
    #endif
  }
  else {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }
}

void Quadcopter::calculateIMUOffsets() {
  for (int i = 0; i < NUMBER_OF_READINGS_IMU_FOR_HEATING; i++) {
    updateAngles();
    delay(25);
  }
  delay(500);
  #ifdef DEBUG_IMU
    Serial.println("Calculating offsets...");
  #endif
  float avgAngles[3] = { 0.0, 0.0, 0.0 };
  for (int i = 0; i < NUMBER_OF_READINGS_IMU; i++) {
    updateAngles();
    avgAngles[0] += getCurrentPitch();
    avgAngles[1] += getCurrentRoll();
    avgAngles[2] += getCurrentYaw();
    delay(25);
  }
  offsetPitch = 0 - (avgAngles[0] / (float) NUMBER_OF_READINGS_IMU);
  offsetRoll = 0 - (avgAngles[1] / (float) NUMBER_OF_READINGS_IMU);
  offsetYaw = 0 - (avgAngles[2] / (float) NUMBER_OF_READINGS_IMU);
  #ifdef DEBUG_IMU
    Serial.print("#Offsets...");
    Serial.print("\tPitch: ");
    Serial.print(offsetPitch);
    Serial.print("\tRoll: ");
    Serial.print(offsetRoll);
    Serial.print("\tYaw: ");
    Serial.println(offsetYaw);
  #endif
}

bool Quadcopter::isCalibrated() {
  for (int i = 0; i < NUMBER_OF_READINGS_IMU_FOR_HEATING; i++) {
    updateAngles();
    delay(20);
  }
  float avgAngles[2] = { 0.0, 0.0 };
  for (int i = 0; i < NUMBER_OF_READINGS_IMU; i++) {
    updateAngles();
    avgAngles[0] += getCurrentPitch();
    avgAngles[1] += getCurrentRoll();
    delay(20);
  }
  avgAngles[0] /= NUMBER_OF_READINGS_IMU;
  avgAngles[1] /= NUMBER_OF_READINGS_IMU;
  #ifdef DEBUG_IMU
    Serial.print("#Checking calibration...");
    Serial.print("\tPitch: ");
    Serial.print(avgAngles[0]);
    Serial.print("\tRoll: ");
    Serial.println(avgAngles[1]);
  #endif
  return (-OFFSET_ANGLE <= avgAngles[0] && avgAngles[0] < OFFSET_ANGLE) && (-OFFSET_ANGLE <= avgAngles[1] && avgAngles[1] < OFFSET_ANGLE);
}

void Quadcopter::calibrateIMU() {
  int numCalibration = 0;
  bool calibrated = false;
  do {
    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
    calibrated = isCalibrated();
    #ifdef DEBUG_IMU
      Serial.print("#Calibration: ");
      Serial.print(numCalibration + 1);
      Serial.print("\tCalibrated: ");
      Serial.println(calibrated);
    #endif
    numCalibration++;
    delay(5000);
  } while (!calibrated && numCalibration <= MAX_CALIBRATION_ATTEMPTS);
}

void Quadcopter::updateAngles() {
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
    myIMU.getAres();

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float) myIMU.accelCount[0] * myIMU.aRes; // - accelBias[0];
    myIMU.ay = (float) myIMU.accelCount[1] * myIMU.aRes; // - accelBias[1];
    myIMU.az = (float) myIMU.accelCount[2] * myIMU.aRes; // - accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
    myIMU.getGres();

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float) myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float) myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float) myIMU.gyroCount[2] * myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
    myIMU.getMres();
    // User environmental x-axis correction in milliGauss, should be
    // automatically calculated
    myIMU.magbias[0] = +470.;
    // User environmental x-axis correction in milliGauss TODO axis??
    myIMU.magbias[1] = +120.;
    // User environmental x-axis correction in milliGauss
    myIMU.magbias[2] = +125.;

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float) myIMU.magCount[0] * myIMU.mRes * myIMU.magCalibration[0] - myIMU.magbias[0];
    myIMU.my = (float) myIMU.magCount[1] * myIMU.mRes * myIMU.magCalibration[1] - myIMU.magbias[1];
    myIMU.mz = (float) myIMU.magCount[2] * myIMU.mRes * myIMU.magCalibration[2] - myIMU.magbias[2];
  }

  // Must be called before updating quaternions!
  myIMU.updateTime();

  //MadgwickQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * PI/180.0f, myIMU.gy * PI/180.0f, myIMU.gz * PI/180.0f,  myIMU.my,  myIMU.mx, myIMU.mz, myIMU.deltat);
  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD, myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my, myIMU.mx, myIMU.mz, myIMU.deltat);

  myIMU.tempCount = myIMU.readTempData();  // Read the adc values
  // Temperature in degrees Centigrade
  myIMU.temperature = ((float) myIMU.tempCount) / 333.87 + 21.0;
  setTemperatureIMU(myIMU.temperature);

  myIMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ() *
                *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1) * *(getQ()+1)
                - *(getQ()+2) * *(getQ()+2) - *(getQ()+3) * *(getQ()+3));
  myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ() *
                *(getQ()+2)));
  myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2) *
                *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1) * *(getQ()+1)
                - *(getQ()+2) * *(getQ()+2) + *(getQ()+3) * *(getQ()+3));
  myIMU.pitch *= RAD_TO_DEG;
  myIMU.yaw   *= RAD_TO_DEG;
  // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
  // 	8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
  // - http://www.ngdc.noaa.gov/geomag-web/#declination
  myIMU.yaw   -= 0.18333;
  myIMU.roll  *= RAD_TO_DEG;

  myIMU.pitch += offsetPitch;
  myIMU.roll += offsetRoll;
  myIMU.yaw += offsetYaw;

  setCurrentPitch(myIMU.pitch);
  setCurrentRoll(myIMU.roll);
  setCurrentYaw(myIMU.yaw);

  #ifdef DEBUG_IMU
    Serial.print("Pitch, Roll, Yaw: ");
    Serial.print(myIMU.pitch, 2);
    Serial.print(", ");
    Serial.print(myIMU.roll, 2);
    Serial.print(", ");
    Serial.println(myIMU.yaw, 2);
  #endif
}

float Quadcopter::getTemperatureIMU() {
  return temperatureIMU;
}

void Quadcopter::setTemperatureIMU(float temperatureIMU) {
  this->temperatureIMU = temperatureIMU;
}

// HC-SR04
int Quadcopter::getDistance() {
   	// To generate a clean pulse we put to LOW 4us
	digitalWrite(HCSR04_TRIGGER_PIN, LOW);
	delayMicroseconds(4);
	// Generate trigger 10us
	digitalWrite(HCSR04_TRIGGER_PIN, HIGH);
	delayMicroseconds(10);
	digitalWrite(HCSR04_TRIGGER_PIN, LOW);

   	// Measure the time between pulses (in microseconds)
 	long duration = pulseIn(HCSR04_ECHO_PIN, HIGH);
	if (duration == 0) {
		pinMode(HCSR04_ECHO_PIN, OUTPUT);
		digitalWrite(HCSR04_ECHO_PIN, LOW);
		delay(1);
		pinMode(HCSR04_ECHO_PIN, INPUT);
	}

	// Convert distance to cm
	int distance = duration * 10 / 292 / 2;

	#ifdef DEBUG_SONAR
		Serial.print("HC-SR04 Floor distance: ");
		Serial.print(distance);
		Serial.println("cm");
	#endif

	return distance;
}

// LED
void Quadcopter::enableLED() {
  digitalWrite(LED_PIN, HIGH);
}

void Quadcopter::disableLED() {
  digitalWrite(LED_PIN, LOW);
}
