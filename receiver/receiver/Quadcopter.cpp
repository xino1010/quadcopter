#include "Quadcopter.h"

Quadcopter::Quadcopter() {

  // LED
  pinMode(LED_PIN , OUTPUT);  //definir pin como salida

  // BMP180
  #ifdef BMP180
    if (!bmp.begin()) {
      #ifdef DEBUG_BMP
        Serial.println(F("Could not find a valid BMP180 sensor, check wiring!"));
      #endif
      while (true) {
        #ifdef DEBUG_BMP
          Serial.println(F("Reset program and connect BMP180 sensor correctly"));
        #endif
        delay(1000);
      }
    }
    delay(1000);
    offsetAltitude = getAltitude();
    previousAltitudeRead = millis();
  #endif

	// PID
	pidPitch = new PID(kpPitch, kiPitch, kdPitch, MIN_PITCH, MAX_PITCH);
	pidRoll = new PID(kpRoll, kiRoll, kdRoll, MIN_ROLL, MAX_ROLL);
	pidYaw = new PID(kpYaw, kiYaw, kdYaw, MIN_YAW, MAX_YAW);
  pidDistance = new PID(kpDistance, kiDistance, kdDistance, MIN_DISTANCE, MAX_DISTANCE);
  pidDistance->setDesiredPoint(DESIRED_DISTANCE);
  pidAltitude = new PID(kpAltitude, kiAltitude, kdAltitude, MIN_ALTITUDE, MAX_ALTITUDE);
  pidAltitude->setDesiredPoint(DESIRED_ALTITUDE);
  lastPitch = 0;
  lastRoll = 0;
  lastYaw = 0;
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
    cm = CONTROL_MODE_ACRO;
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
		Serial.print(F("BMP180 Altitude: "));
		Serial.print(altitude);
		Serial.println(F("m"));
	#endif
	return altitude;
}

float Quadcopter::getTemperature() {
	float temperature = bmp.readTemperature();;
	#ifdef DEBUG_BMP
		Serial.print(F("BMP180 Temperature: "));
		Serial.print(temperature);
		Serial.println(F("ºC"));
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
    Serial.println(F("Motors attached"));
  #endif
}

void Quadcopter::armMotors() {
	motorFL.writeMicroseconds(ARM_MOTOR);
	motorFR.writeMicroseconds(ARM_MOTOR);
	motorBL.writeMicroseconds(ARM_MOTOR);
	motorBR.writeMicroseconds(ARM_MOTOR);

  #ifdef DEBUG_MOTORS
    Serial.println(F("Motors armed"));
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
    tmpVFL = getThrottle() - resultRoll - resultPitch + resultYaw;
  	tmpVFR = getThrottle() + resultRoll - resultPitch - resultYaw;
  	tmpVBL = getThrottle() - resultRoll + resultPitch - resultYaw;
  	tmpVBR = getThrottle() + resultRoll + resultPitch + resultYaw;
  }
  else if (getControlMode() == CONTROL_MODE_HOLD_DISTANCE) {
    int dist = getDistance();
    pidDistance->setCurrentPoint(dist);
    double resultDistance = pidDistance->calculate();
    resultDistance = map(resultDistance, MIN_DISTANCE, MAX_DISTANCE, MIN_VALUE_PID, MAX_VALUE_PID);
    tmpVFL = MIN_VALUE_MOTOR - resultDistance - resultRoll - resultPitch + resultYaw;
  	tmpVFR = MIN_VALUE_MOTOR - resultDistance + resultRoll - resultPitch - resultYaw;
  	tmpVBL = MIN_VALUE_MOTOR - resultDistance - resultRoll + resultPitch - resultYaw;
  	tmpVBR = MIN_VALUE_MOTOR - resultDistance + resultRoll + resultPitch + resultYaw;
  }
  else if (getControlMode() == CONTROL_MODE_HOLD_ALTITUDE) {
    int alt = getAltitude();
    double currentPointAltitude = alt - offsetAltitude;
    pidAltitude->setCurrentPoint(currentPointAltitude);
    double resultAltitude = pidAltitude->calculate();
    resultAltitude = map(resultAltitude, MIN_ALTITUDE, MAX_ALTITUDE, MIN_VALUE_PID, MAX_VALUE_PID);
    tmpVFL = MIN_VALUE_MOTOR - resultAltitude - resultRoll - resultPitch + resultYaw;
  	tmpVFR = MIN_VALUE_MOTOR - resultAltitude + resultRoll - resultPitch - resultYaw;
  	tmpVBL = MIN_VALUE_MOTOR - resultAltitude - resultRoll + resultPitch - resultYaw;
  	tmpVBR = MIN_VALUE_MOTOR - resultAltitude + resultRoll + resultPitch + resultYaw;
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
    Serial.print(F("CM: "));
    Serial.print(cm);
    Serial.print(F(" FL: "));
    Serial.print(getVelocityFL());
    Serial.print(F(" FR: "));
    Serial.print(getVelocityFR());
    Serial.print(F(" BL: "));
    Serial.print(getVelocityBL());
    Serial.print(F(" BR: "));
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
        Serial.print(F("kP: "));
        Serial.print(pidPitch->getKp());
        Serial.print(F("\tkI: "));
        Serial.print(pidPitch->getKi());
        Serial.print(F("\tkD: "));
        Serial.print(pidPitch->getKd());
        Serial.print(F("\tReset: "));
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
        Serial.print(F("kP: "));
        Serial.print(pidRoll->getKp());
        Serial.print(F("\tkI: "));
        Serial.print(pidRoll->getKi());
        Serial.print(F("\tkD: "));
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
        Serial.print(F("kP: "));
        Serial.print(pidYaw->getKp());
        Serial.print(F("\tkI: "));
        Serial.print(pidYaw->getKi());
        Serial.print(F("\tkD: "));
        Serial.println(pidYaw->getKd());
      #endif
    #endif

    disableLED();
  }
  else {
    #ifdef DEBUG_RADIO
      Serial.println(F("There is no radio pid data"));
    #endif
  }
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

void Quadcopter::calculateOffsets() {
  for (int i = 0; i < NORMALIZE_SAMPLES; i++) {
    updateAngles();
  }
  float offsets[3] = {0, 0, 0};
  for (int i = 0; i < OFFSET_SAMPLES; i++) {
    updateAngles();
    offsets[0] += getCurrentPitch();
    offsets[1] += getCurrentRoll();
    offsets[2] += getCurrentYaw();
  }
  offsetPitch = offsets[0] / OFFSET_SAMPLES;
  offsetRoll = offsets[1] / OFFSET_SAMPLES;
  offsetYaw = offsets[2] / OFFSET_SAMPLES;
  #ifdef DEBUG_IMU
    Serial.print(F("Offset pitch: "));
    Serial.print(offsetPitch);
    Serial.print(F("\tOffset roll: "));
    Serial.print(offsetRoll);
    Serial.print(F("\tOffset yaw: "));
    Serial.println(offsetYaw);
  #endif
}

void Quadcopter::initIMU() {
  // initialize device
  #ifdef DEBUG_IMU
    Serial.println(F("Initializing I2C devices..."));
  #endif
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  #ifdef DEBUG_IMU
  // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
  #endif
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  /*
  mpu.setXGyroOffset(27);
  mpu.setYGyroOffset(-34);
  mpu.setZGyroOffset(13);
  mpu.setXAccelOffset(-3074);
  mpu.setYAccelOffset(-1648);
  mpu.setZAccelOffset(1450);
  */
  mpu.setXGyroOffset(3);
  mpu.setYGyroOffset(1);
  mpu.setZGyroOffset(1);
  //mpu.setXAccelOffset(14);
  //mpu.setYAccelOffset(10);
  mpu.setZAccelOffset(16386);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
      // turn on the DMP, now that it's ready
      #ifdef DEBUG_IMU
        Serial.println(F("Enabling DMP..."));
      #endif
      mpu.setDMPEnabled(true);

      // enable Arduino interrupt detection
      #ifdef DEBUG_IMU
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
      #endif
      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      #ifdef DEBUG_IMU
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
      #endif
      dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      #ifdef DEBUG_IMU
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
      #endif
  }
}

void Quadcopter::updateAngles() {

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) { }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      #ifdef DEBUG_IMU
        Serial.println(F("FIFO overflow!"));
      #endif
  }
  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);

      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      float pitch = ypr[1] * (float) (180.0/M_PI);
      float roll = ypr[2] * (float) (180.0/M_PI);
      float yaw = ypr[0] * (float) (180.0/M_PI);
      if (yaw < 0) {
        yaw += 360.0;
      }

      float currentPitch = pitch - offsetPitch;
      float currentRoll = roll - offsetRoll;
      float currentYaw = yaw;

      if (myAbs(currentPitch - lastPitch) > ALLOWED_VARIATION_PITCH) {
        #ifdef DEBUG_IMU
          Serial.print("Abnormal Pitch angle. Last: ");
          Serial.print(lastPitch);
          Serial.print("\tCurrent: ");
          Serial.println(currentPitch); 
        #endif
        currentPitch = lastPitch;
      }
      if (myAbs(currentRoll - lastRoll) > ALLOWED_VARIATION_ROLL) {
        #ifdef DEBUG_IMU
          Serial.print("Abnormal Roll angle. Last: ");
          Serial.print(lastRoll);
          Serial.print("\tCurrent: ");
          Serial.println(currentRoll); 
        #endif
        currentRoll = lastRoll;
      }
      if (myAbs(currentYaw - lastYaw) > ALLOWED_VARIATION_YAW) {
        /*
        #ifdef DEBUG_IMU
          Serial.print("Abnormal Yaw angle. Last: ");
          Serial.print(lastYaw);
          Serial.print("\tCurrent: ");
          Serial.println(currentYaw); 
        #endif
        */
        currentYaw = lastYaw;
      }

      lastPitch = currentPitch;
      lastRoll = currentRoll;
      lastYaw = currentYaw;

      setCurrentPitch(currentPitch);
      setCurrentRoll(currentRoll);
      setCurrentYaw(yaw);

      mpu.resetFIFO();

      #ifdef DEBUG_IMU
        Serial.print("Pitch, Roll, Yaw: ");
        Serial.print(getCurrentPitch());
        Serial.print(", ");
        Serial.print(getCurrentRoll());
        Serial.print(", ");
        Serial.println(getCurrentYaw());
      #endif
  }
}

bool Quadcopter::getDmpReady() {
  return dmpReady;
}

void Quadcopter::dmpDataReady() {
  mpuInterrupt = true;
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
		Serial.print(F("HC-SR04 Floor distance: "));
		Serial.print(distance);
		Serial.println(F("cm"));
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
