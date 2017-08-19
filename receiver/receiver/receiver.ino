#include <Adafruit_BMP085.h>
#include <Servo.h>
#include <nRF24L01.h>
#include "RF24.h"
#include "PID.h"
#include <Wire.h>

#define DEBUG
#ifdef DEBUG
  //#define DEBUG_BMP
  #define DEBUG_IMU
  //#define DEBUG_MOTORS
  //#define DEBUG_RADIO
  //#define DEBUG_SONAR
  //#define DEBUG_PID
#endif

//#define NORMAL_MODE
#define CALIBRATION_MODE

#ifdef CALIBRATION_MODE
  #define CALIBRATION_PITCH
  //#define CALIBRATION_ROLL
  //#define CALIBRATION_YAW
#endif

// BMP180
//#define BMP180
#define MIN_ALTITUDE 200
#define MAX_ALTITUDE 5000
#define TIME_READ_ALTITUDE 3000
#define DESIRED_ALTITUDE 2000

// MOTORS
#define PIN_MOTOR_FL 3
#define PIN_MOTOR_FR 5
#define PIN_MOTOR_BL 6
#define PIN_MOTOR_BR 9
#define ARM_MOTOR 800
#define ZERO_VALUE_MOTOR 900
#define MIN_VALUE_MOTOR 1000
#define MAX_VALUE_MOTOR 2000
#define MIN_VALUE_PID -1000.0
#define MAX_VALUE_PID 1000.0

// RADIO
const byte radioAddress[5] = {'c', 'a', 'n', 'a', 'l'};
#define NFR24L01_CE 8
#define NFR24L01_CSN 10
#define CONTROL_MODE_OFF 100
#define CONTROL_MODE_ACRO 101
#define CONTROL_MODE_HOLD_DISTANCE 102
#define CONTROL_MODE_HOLD_ALTITUDE 103

// HC-SR04
#define HCSR04_ECHO_PIN A3
#define HCSR04_TRIGGER_PIN 4
#define MIN_DISTANCE 0
#define MAX_DISTANCE MIN_ALTITUDE
#define DESIRED_DISTANCE 200

// IMU
#define MAX_CHANGE_PITCH 15
#define MIN_PITCH -30
#define MAX_PITCH 30
#define MAX_CHANGE_ROLL 15
#define MIN_ROLL -30
#define MAX_ROLL 30
#define MAX_CHANGE_YAW 20
#define MIN_YAW -90
#define MAX_YAW 90
#define READS_OFFSETS 2000
#define HEAT_OFFSETS 400

// LED
#define LED_PIN 7

// BMP180
Adafruit_BMP085 bmp;

// PID's
double kpPitch = 0, kiPitch = 0, kdPitch = 0;
double kpRoll = 0, kiRoll = 0, kdRoll = 0;
double kpYaw = 0, kiYaw = 0, kdYaw = 0;
double kpDistance = 0, kiDistance = 0, kdDistance = 0;
double kpAltitude = 0, kiAltitude = 0, kdAltitude = 0;
PID *pidRoll, *pidPitch, *pidYaw, *pidDistance, *pidAltitude;

// MOTORS
int vFL, vFR, vBL, vBR;
Servo motorFL, motorFR, motorBL, motorBR;

// RADIO
int throttle;
float lastDesiredPitch, lastDesiredRoll, lastDesiredYaw;
float desiredPitch, desiredRoll, desiredYaw;
RF24 *radio;
int radioData[7];
const int sizeRadioData = sizeof(radioData);
float radioPIDdata[4];
const int sizeRadioPIDdata = sizeof(radioPIDdata);
int cm;

// IMU
int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ, Tmp;
float Acceleration_angle[3];
float Gyro_angle[3];
float Total_angle[3];
float offsets[2] = {0.0, 0.0};
float elapsedTime, time, timePrev;
float rad_to_deg = 180/3.141592654;
float currentPitch, currentRoll, currentYaw;

int myAbs(int value) {
  return value > 0 ? value : value * -1;
}

// BMP180
float getAltitude() {
  float altitude = bmp.readAltitude();
  #ifdef DEBUG_BMP
    Serial.print(F("BMP180 Altitude: "));
    Serial.print(altitude);
    Serial.println(F("m"));
  #endif
  return altitude;
}

float getTemperature() {
  float temperature = bmp.readTemperature();;
  #ifdef DEBUG_BMP
    Serial.print(F("BMP180 Temperature: "));
    Serial.print(temperature);
    Serial.println(F("ºC"));
  #endif
  return temperature;
}

// MOTORS
void connectMotors() {
  motorFL.attach(PIN_MOTOR_FL);
  motorFR.attach(PIN_MOTOR_FR);
  motorBL.attach(PIN_MOTOR_BL);
  motorBR.attach(PIN_MOTOR_BR);

  #ifdef DEBUG_MOTORS
    Serial.println(F("Motors attached"));
  #endif
}

void armMotors() {
  motorFL.writeMicroseconds(ARM_MOTOR);
  motorFR.writeMicroseconds(ARM_MOTOR);
  motorBL.writeMicroseconds(ARM_MOTOR);
  motorBR.writeMicroseconds(ARM_MOTOR);

  #ifdef DEBUG_MOTORS
    Serial.println(F("Motors armed"));
  #endif
}

void calculateVelocities() {

  // Update current points
  pidPitch->setCurrentPoint(currentPitch);
  pidRoll->setCurrentPoint(currentRoll);
  pidYaw->setCurrentPoint(currentYaw);

  // Calculate the pids from the setpoints and the current point
  double resultPitch = pidPitch->calculate();
  double resultRoll = pidRoll->calculate();
  double resultYaw = pidYaw->calculate();

  resultPitch = map(resultPitch, MIN_PITCH, MAX_PITCH, MIN_VALUE_PID, MAX_VALUE_PID);
  resultRoll = map(resultRoll, MIN_ROLL, MAX_ROLL, MIN_VALUE_PID, MAX_VALUE_PID);
  resultYaw = map(resultYaw, MIN_YAW, MAX_YAW, MIN_VALUE_PID, MAX_VALUE_PID);

  int tmpVFL = 0;
  int tmpVFR = 0;
  int tmpVBL = 0;
  int tmpVBR = 0;

  if (cm == CONTROL_MODE_ACRO) {
    tmpVFL = throttle - resultRoll + resultPitch + resultYaw;
    tmpVFR = throttle + resultRoll + resultPitch - resultYaw;
    tmpVBL = throttle - resultRoll - resultPitch - resultYaw;
    tmpVBR = throttle + resultRoll - resultPitch + resultYaw;
  }
  else if (cm == CONTROL_MODE_HOLD_DISTANCE) {
    int dist = getDistance();
    pidDistance->setCurrentPoint(dist);
    double resultDistance = pidDistance->calculate();
    resultDistance = map(resultDistance, MIN_DISTANCE, MAX_DISTANCE, MIN_VALUE_PID, MAX_VALUE_PID);
    tmpVFL = MIN_VALUE_MOTOR - resultDistance - resultRoll + resultPitch + resultYaw;
    tmpVFR = MIN_VALUE_MOTOR - resultDistance + resultRoll + resultPitch - resultYaw;
    tmpVBL = MIN_VALUE_MOTOR - resultDistance - resultRoll - resultPitch - resultYaw;
    tmpVBR = MIN_VALUE_MOTOR - resultDistance + resultRoll - resultPitch + resultYaw;
  }
  else if (cm == CONTROL_MODE_HOLD_ALTITUDE) {
    #ifdef BMP180
      int alt = getAltitude();
      double currentPointAltitude = alt - offsetAltitude;
      pidAltitude->setCurrentPoint(currentPointAltitude);
      double resultAltitude = pidAltitude->calculate();
      resultAltitude = map(resultAltitude, MIN_ALTITUDE, MAX_ALTITUDE, MIN_VALUE_PID, MAX_VALUE_PID);
      tmpVFL = MIN_VALUE_MOTOR - resultAltitude - resultRoll + resultPitch + resultYaw;
      tmpVFR = MIN_VALUE_MOTOR - resultAltitude + resultRoll + resultPitch - resultYaw;
      tmpVBL = MIN_VALUE_MOTOR - resultAltitude - resultRoll - resultPitch - resultYaw;
      tmpVBR = MIN_VALUE_MOTOR - resultAltitude + resultRoll - resultPitch + resultYaw;
    #endif
  }

  // Check that the speeds do not exceed the limits
  vFL = constrain(tmpVFL, MIN_VALUE_MOTOR, MAX_VALUE_MOTOR);
  vFR = constrain(tmpVFR, MIN_VALUE_MOTOR, MAX_VALUE_MOTOR);
  vBL = constrain(tmpVBL, MIN_VALUE_MOTOR, MAX_VALUE_MOTOR);
  vBR = constrain(tmpVBR, MIN_VALUE_MOTOR, MAX_VALUE_MOTOR);
}

void updateMotorsVelocities() {
  #ifdef DEBUG_MOTORS
    Serial.print(F("CM: "));
    Serial.print(cm);
    Serial.print(F(" FL: "));
    Serial.print(vFL);
    Serial.print(F(" FR: "));
    Serial.print(vFR);
    Serial.print(F(" BL: "));
    Serial.print(vBL);
    Serial.print(F(" BR: "));
    Serial.println(vBR);
  #endif

  motorFL.writeMicroseconds(vFL);
  motorFR.writeMicroseconds(vFR);
  motorBL.writeMicroseconds(vBL);
  motorBR.writeMicroseconds(vBR);
}

void stopMotors() {
  vFL = ZERO_VALUE_MOTOR;
  vFR = ZERO_VALUE_MOTOR;
  vBL = ZERO_VALUE_MOTOR;
  vBR = ZERO_VALUE_MOTOR;

  #ifdef BMP180
    // Update altitude because quadcopter could have changed the position
    if (millis() - previousAltitudeRead > TIME_READ_ALTITUDE) {
      offsetAltitude = getAltitude();
      previousAltitudeRead = millis();
    }
  #endif
}

// RADIO
void updateRadioInfo() {
  if (radio->available()) {
    enableLED();
    radio->read(radioData, sizeRadioData);

    throttle = radioData[0];
    // Update desired angles
    // Pitch 
    desiredPitch = radioData[1];
    pidPitch->setDesiredPoint(radioData[1]);
    // Roll
    desiredRoll = radioData[2];
    pidRoll->setDesiredPoint(desiredRoll);
    // Yaw
    desiredYaw = radioData[3];
    pidYaw->setDesiredPoint(desiredYaw);

    // Check if there have sent any abnormal data
    if (myAbs(lastDesiredPitch - desiredPitch) > MAX_CHANGE_PITCH) {
      desiredPitch = lastDesiredPitch;
    }
    if (myAbs(lastDesiredRoll - desiredRoll) > MAX_CHANGE_ROLL) {
      desiredRoll = lastDesiredRoll;
    }
    if (myAbs(lastDesiredYaw - desiredYaw) > MAX_CHANGE_YAW) {
      desiredYaw = lastDesiredYaw;
    }

    // Update last desired angle
    lastDesiredPitch = desiredPitch;
    lastDesiredRoll = desiredRoll;
    lastDesiredYaw = desiredYaw;

    // Power off motors
    if (radioData[4] == LOW) {
      cm = CONTROL_MODE_OFF;
    }
    // Stabilize dron and keep the distance
    else if (radioData[5] == HIGH) {
      float distance = getDistance();
      if (distance >= MIN_DISTANCE && distance < MAX_DISTANCE) {
        cm = CONTROL_MODE_HOLD_DISTANCE;
      }
      else {
        cm = CONTROL_MODE_ACRO;
      }
    }
    // Stabilize dron and keep the altitud
    else if (radioData[6] == HIGH) {
      #ifdef BMP180
        float altitudeSea = getAltitude();
        float currentAltitude = altitudeSea - offsetAltitude;
        if (currentAltitude >= MIN_ALTITUDE && currentAltitude < MAX_ALTITUDE) {
          cm = CONTROL_MODE_HOLD_ALTITUDE;
        }
        else {
          cm = CONTROL_MODE_ACRO;
        }
      #endif
    }
    // Stabilize dron and respond transmitter orders
    else {
      cm = CONTROL_MODE_ACRO;
    }
    disableLED();
  }
}

void updatePIDInfo() {
  if (radio->available()) {
    enableLED();
    radio->read(radioPIDdata, sizeRadioPIDdata);
    int resetPid = (int) radioPIDdata[3];

    if (resetPid) {
      cm = CONTROL_MODE_OFF;
    }
    else {
      cm = CONTROL_MODE_ACRO;
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
void initIMU() {
  Wire.begin();
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  #ifdef DEBUG_IMU
    Serial.println(F("MPU6050 initialized"));
  #endif
}

// HC-SR04
int getDistance() {
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
void enableLED() {
  digitalWrite(LED_PIN, HIGH);
}

void disableLED() {
  digitalWrite(LED_PIN, LOW);
}


// IMU
void getAngles(bool useOffsets) {
  timePrev = time;  // the previous time is stored before the actual time read
  time = millis();  // actual time read
  elapsedTime = (time - timePrev) / 1000;
   
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); //Ask for the 0x3B register- correspond to AcX
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,14,true);     
  Acc_rawX = Wire.read() << 8 | Wire.read(); //each value needs two registres
  Acc_rawY = Wire.read() << 8 | Wire.read();
  Acc_rawZ = Wire.read() << 8 | Wire.read();
  Tmp = Wire.read() << 8 | Wire.read();
  Gyr_rawX = Wire.read() << 8 | Wire.read(); //Once again we shif and sum
  Gyr_rawY = Wire.read() << 8 | Wire.read();
  Gyr_rawZ = Wire.read() << 8 | Wire.read();
     
  /*---X---*/
  Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
  /*---Y---*/
  Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
  /*---X---*/
  Gyro_angle[0] = Gyr_rawX / 131.0; 
  /*---Y---*/
  Gyro_angle[1] = Gyr_rawY / 131.0;
  /*---Z---*/
  Gyro_angle[2] = Gyr_rawZ / 131.0;

  /*---X axis angle---*/
  Total_angle[0] = 0.98 * (Total_angle[0] + Gyro_angle[0] * elapsedTime) + 0.02 * Acceleration_angle[0];
  /*---Y axis angle---*/
  Total_angle[1] = 0.98 * (Total_angle[1] + Gyro_angle[1] * elapsedTime) + 0.02 * Acceleration_angle[1];
  /*---Z axis angles---*/
  //Total_angle[2] = Total_angle[2] + Gyro_angle[2] * elapsedTime;
  Total_angle[2] = Gyro_angle[2];

  if (useOffsets) {
    currentPitch = Total_angle[1] + offsets[1];
    currentRoll = Total_angle[0] + offsets[0];
    currentYaw = Total_angle[2];
  }
  else {
    currentPitch = Total_angle[1];
    currentRoll = Total_angle[0];
    currentYaw = Total_angle[2];
  }
  
  #ifdef DEBUG_IMU
    Serial.print("Pitch: ");
    Serial.print(currentPitch);
    Serial.print("\tRoll: ");
    Serial.print(currentRoll);
    Serial.print("\tYaw: ");
    Serial.println(currentYaw);
  #endif
}

void calculateOffsets() {
  for (int i = 0; i < READS_OFFSETS; i++) {
    getAngles(false);
    if (i > HEAT_OFFSETS) {
      offsets[0] += (0 - Total_angle[0]);
      offsets[1] += (0 - Total_angle[1]);
    }
  }
  offsets[0] /= (float) (READS_OFFSETS - HEAT_OFFSETS);
  offsets[1] /= (float) (READS_OFFSETS - HEAT_OFFSETS);
  Serial.println("******************************************************************************");
  Serial.print("OffsetX: ");
  Serial.print(offsets[0]);
  Serial.print("\tOffsetY: ");
  Serial.println(offsets[1]);
  Serial.println("******************************************************************************");
}

void initMPU() {
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

void initVars() {
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
    pidAltitude = new PID(kpAltitude, kiAltitude, kdAltitude, MIN_ALTITUDE, MAX_ALTITUDE);
    pidAltitude->setDesiredPoint(DESIRED_ALTITUDE);
    pidAltitude->setDesiredPoint(offsetAltitude);
  #endif

  // PID
  pidPitch = new PID(kpPitch, kiPitch, kdPitch, MIN_PITCH, MAX_PITCH);
  pidRoll = new PID(kpRoll, kiRoll, kdRoll, MIN_ROLL, MAX_ROLL);
  pidYaw = new PID(kpYaw, kiYaw, kdYaw, MIN_YAW, MAX_YAW);
  pidDistance = new PID(kpDistance, kiDistance, kdDistance, MIN_DISTANCE, MAX_DISTANCE);
  pidDistance->setDesiredPoint(DESIRED_DISTANCE);
  
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

  radio->begin();
  radio->setDataRate(RF24_250KBPS);
  radio->setPALevel(RF24_PA_MAX); // RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
  radio->openReadingPipe(1, radioAddress);
  radio->startListening();

  // IMU
  currentPitch = 0;
  currentRoll = 0;
  currentYaw = 0;
  
  // HC-SR04
  //pinMode(HCSR04_ECHO_PIN, INPUT);
  //pinMode(HCSR04_TRIGGER_PIN, OUTPUT);

}

void setup() {
  Wire.begin();
  
  #ifdef DEBUG
    Serial.begin(250000);
    while (!Serial) {}
  #endif
  
  initVars();

  // Init MPU9250
  initMPU();

  delay(500);

  time = millis(); //Start counting time in milliseconds
  calculateOffsets();
  enableLED();

  // Connect Motors
  connectMotors();
  delay(50);

  // Send a minimum signal to prepare the motors
  armMotors();
  delay(50);

  int i = 9;
  #ifdef DEBUG
    Serial.println("Countdown: ");
  #endif
  while (i >= 0) {
    delay(500);
    disableLED();
    #ifdef DEBUG
      Serial.print(i);
      Serial.println("...");
    #endif
    delay(500);
    enableLED();
    i--;
  }
	#ifdef DEBUG
	  Serial.println("Quadcopter initialized");
  #endif

  disableLED();
  
}

void loop() {

  #ifdef NORMAL_MODE
  	// Read data from radio
  	updateRadioInfo();
  #endif

  #ifdef CALIBRATION_MODE
    // Read constants of kpi from radio
    updatePIDInfo();
  #endif

  // Read angles from sensor
  getAngles(true);

  if (cm == CONTROL_MODE_OFF) {
    // Set minim velocity to all motors
    stopMotors();
  }
  else {
    // Calculate velocities of each motor depending of ControlMode
    calculateVelocities();
  }

  // Send velocities to motors
  updateMotorsVelocities();

}
