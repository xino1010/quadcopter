#include <Adafruit_BMP085.h>
#include <Servo.h>
#include <nRF24L01.h>
#include "RF24.h"
#include <PID_v1.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>

#define DEBUG
#ifdef DEBUG
  //#define DEBUG_BMP
  #define DEBUG_IMU
  //#define DEBUG_MOTORS
  //#define DEBUG_RADIO
  //#define DEBUG_PID
  //#define DEBUG_SONAR
#endif

#define NORMAL_MODE
//#define CALIBRATION_MODE

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
#define ZERO_VALUE_MOTOR 1000
#define MIN_VALUE_MOTOR 1100
#define MAX_VALUE_MOTOR 1500

// RADIO
#define THROTTLE_MIN 512
#define THROTTLE_MAX 1023
// R_PITCH
#define PITCH_RMIN 0
#define PITCH_RMEDIUM 512
#define PITCH_RMAX 1024
#define PITCH_WMIN -30
#define PITCH_WMAX 30
#define ROFFSET_PITCH 20
// R_ROLL
#define ROLL_RMIN 0
#define ROLL_RMEDIUM 512
#define ROLL_RMAX 1024
#define ROLL_WMIN -30
#define ROLL_WMAX 30
#define ROFFSET_ROLL 20

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
#define HEAT_MPU_SECONDS 12000
#define OFFSETS_MPU_SECONDS 2500
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

// LED
#define LED_PIN 7
#define SECONDS_COUNTDOWN 5

// Track loop time.
unsigned long prev_time = 0;

// BMP180
Adafruit_BMP085 bmp;

// PID
// Pitch
#define PITCH_PID_MIN -45 
#define PITCH_PID_MAX 45
float KP_PITCH = 3;
float KI_PITCH = 0;
float KD_PITCH = 0;
double pidPitchIn, pidPitchOut, pidPitchSetpoint = 0;
PID pidPitch(&pidPitchIn, &pidPitchOut, &pidPitchSetpoint, KP_PITCH, KI_PITCH, KD_PITCH, DIRECT);

// Roll
#define ROLL_PID_MIN -45 
#define ROLL_PID_MAX 45
float KP_ROLL = KP_PITCH;
float KI_ROLL = KI_PITCH;
float KD_ROLL = KD_PITCH;
double pidRollIn, pidRollOut, pidRollSetpoint = 0;
PID pidRoll(&pidRollIn, &pidRollOut, &pidRollSetpoint, KP_ROLL, KI_ROLL, KD_ROLL, DIRECT);

// MOTORS
int velocityFL, velocityFR, velocityBL, velocityBR;
Servo motorFL, motorFR, motorBL, motorBR;

// RADIO
int throttle;
RF24 *radio;
int radioData[7];
const int sizeRadioData = sizeof(radioData);
float radioPIDdata[4];
const int sizeRadioPIDdata = sizeof(radioPIDdata);
int cm;

// IMU
float offsetPitch, offsetRoll, offsetYaw = 0;
float anglePitch, angleRoll, angleYaw = 0;
MPU6050 mpu;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void printFrequency() {
  #ifdef DEBUGa
    unsigned long elapsed_time = micros() - prev_time;
    Serial.print(F("Time:"));
    Serial.print((float) elapsed_time / 1000);
    Serial.println(F("ms"));
  #endif
}

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
  motorFL.writeMicroseconds(ZERO_VALUE_MOTOR);
  motorFR.writeMicroseconds(ZERO_VALUE_MOTOR);
  motorBL.writeMicroseconds(ZERO_VALUE_MOTOR);
  motorBR.writeMicroseconds(ZERO_VALUE_MOTOR);

  #ifdef DEBUG_MOTORS
    Serial.println(F("Motors armed"));
  #endif
}

void calculateVelocities() {

  pidPitchIn = anglePitch;
  pidRollIn = angleRoll;
  pidPitch.Compute();
  pidRoll.Compute();

  switch (cm) {
    case CONTROL_MODE_OFF:
      // Stop motors
      velocityFL = ZERO_VALUE_MOTOR;
      velocityFR = ZERO_VALUE_MOTOR;
      velocityBL = ZERO_VALUE_MOTOR;
      velocityBR = ZERO_VALUE_MOTOR;
      break;
    case CONTROL_MODE_ACRO:
      velocityFL = throttle + pidPitchOut + pidRollOut;
      velocityFR = throttle + pidPitchOut - pidRollOut;
      velocityBL = throttle - pidPitchOut + pidRollOut;
      velocityBR = throttle - pidPitchOut - pidRollOut;

      velocityFL = constrain(velocityFL, MIN_VALUE_MOTOR, MAX_VALUE_MOTOR);
      velocityFR = constrain(velocityFR, MIN_VALUE_MOTOR, MAX_VALUE_MOTOR);
      velocityBL = constrain(velocityBL, MIN_VALUE_MOTOR, MAX_VALUE_MOTOR);
      velocityBR = constrain(velocityBR, MIN_VALUE_MOTOR, MAX_VALUE_MOTOR);
      break;
    case CONTROL_MODE_HOLD_DISTANCE:
      velocityFL = constrain(velocityFL, MIN_VALUE_MOTOR, MAX_VALUE_MOTOR);
      velocityFR = constrain(velocityFR, MIN_VALUE_MOTOR, MAX_VALUE_MOTOR);
      velocityBL = constrain(velocityBL, MIN_VALUE_MOTOR, MAX_VALUE_MOTOR);
      velocityBR = constrain(velocityBR, MIN_VALUE_MOTOR, MAX_VALUE_MOTOR);
      break;
    case CONTROL_MODE_HOLD_ALTITUDE:
      velocityFL = constrain(velocityFL, MIN_VALUE_MOTOR, MAX_VALUE_MOTOR);
      velocityFR = constrain(velocityFR, MIN_VALUE_MOTOR, MAX_VALUE_MOTOR);
      velocityBL = constrain(velocityBL, MIN_VALUE_MOTOR, MAX_VALUE_MOTOR);
      velocityBR = constrain(velocityBR, MIN_VALUE_MOTOR, MAX_VALUE_MOTOR);
      break;
    default:
      // Stop motors
      velocityFL = ZERO_VALUE_MOTOR;
      velocityFR = ZERO_VALUE_MOTOR;
      velocityBL = ZERO_VALUE_MOTOR;
      velocityBR = ZERO_VALUE_MOTOR;
      break;
  }

}

void updateMotorsVelocities() {
  #ifdef DEBUG_MOTORS
    Serial.print(F("CM: "));
    Serial.print(cm);
    Serial.print(F(" VFL: "));
    Serial.print(velocityFL);
    Serial.print(F(" VFR: "));
    Serial.print(velocityFR);
    Serial.print(F(" VBL: "));
    Serial.print(velocityBL);
    Serial.print(F(" VBR: "));
    Serial.println(velocityBR);
  #endif

  motorFL.writeMicroseconds(velocityFL);
  motorFR.writeMicroseconds(velocityFR);
  motorBL.writeMicroseconds(velocityBL);
  motorBR.writeMicroseconds(velocityBR);
}

// RADIO
void updateRadioInfo() {
  if (radio->available()) {
    enableLED();
    radio->read(radioData, sizeRadioData);

    // R_THROTTLE
    throttle = map(radioData[0], THROTTLE_MIN, THROTTLE_MAX, MIN_VALUE_MOTOR, MAX_VALUE_MOTOR);
    throttle = constrain(throttle, MIN_VALUE_MOTOR, MAX_VALUE_MOTOR);
    // R_PITCH
    if (radioData[1] >= PITCH_RMEDIUM - ROFFSET_PITCH && radioData[1] <= PITCH_RMEDIUM + ROFFSET_PITCH) {
      pidPitchSetpoint = 0;
    }
    else {
      pidPitchSetpoint = map(radioData[1], PITCH_RMIN, PITCH_RMAX, PITCH_WMIN, PITCH_WMAX);
    }
    // R_ROLL
    if (radioData[2] >= ROLL_RMEDIUM - ROFFSET_ROLL && radioData[2] <= ROLL_RMEDIUM + ROFFSET_ROLL) {
      pidRollSetpoint = 0;
    }
    else {
      pidRollSetpoint = map(radioData[2], ROLL_RMIN, ROLL_RMAX, ROLL_WMIN, ROLL_WMAX);
    }

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
        if (altitudeSea >= MIN_ALTITUDE && altitudeSea < MAX_ALTITUDE) {
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

    #ifdef DEBUG_RADIO
      Serial.print(F("Throttle: "));
      Serial.print(throttle);
      Serial.print(F("\t\tPitch: "));
      Serial.print(pidPitchSetpoint);
      Serial.print(F("\t\tRoll: "));
      Serial.print(pidRollSetpoint);
      Serial.print(F("\t\tYaw: "));
      Serial.print(pidYawSetpoint);
      Serial.print(F("\t\tCM: "));
      Serial.println(cm);
    #endif
    
    disableLED();
  }
  else {
    #ifdef DEBUG_RADIO
      //Serial.println(F("There is not radio data"));
    #endif
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
      pidPitch.SetTunings(radioPIDdata[0], radioPIDdata[1], radioPIDdata[2]);
      #ifdef DEBUG_PID
        Serial.print(F("PITCH - kP: "));
        Serial.print(pidPitch.GetKp());
        Serial.print(F("\t\tkI: "));
        Serial.print(pidPitch.GetKi());
        Serial.print(F("\t\tkD: "));
        Serial.print(pidPitch.GetKd());
        Serial.print(F("\t\tReset: "));
        Serial.println(resetPid);
      #endif
    #endif

    #ifdef CALIBRATION_ROLL
      pidRoll.SetTunings(radioPIDdata[0], radioPIDdata[1], radioPIDdata[2]);
      #ifdef DEBUG_PID
        Serial.print(F("ROLL - kP: "));
        Serial.print(pidRoll.GetKp());
        Serial.print(F("\t\tkI: "));
        Serial.print(pidRoll.GetKi());
        Serial.print(F("\t\tkD: "));
        Serial.print(pidRoll.GetKd());
        Serial.print(F("\t\tReset: "));
        Serial.println(resetPid);
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

  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {}

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
  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
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

      anglePitch = ypr[1] * 180/M_PI; // Pitch
      angleRoll = ypr[2] * 180/M_PI; // Roll
      angleYaw = ypr[0] * 180/M_PI; // Roll
      if (angleYaw < 0) {
        angleYaw += 360.0;
      }
    
      if (useOffsets) {
        anglePitch += offsetPitch;
        angleRoll += offsetRoll;
        angleYaw += offsetYaw;
      }
    
      #ifdef DEBUG_IMU
        Serial.print(F("Pitch: "));
        Serial.print(anglePitch);
        Serial.print(F("\t\tRoll: "));
        Serial.print(angleRoll);
        Serial.print(F("\t\tYaw: "));
        Serial.println(angleYaw);
      #endif
  }
}

void calculateOffsets() {
  #ifdef DEBUG_IMU
    Serial.println(F("Heating mpu6050..."));
  #endif
  unsigned long currentTime = millis();
  while (millis() - currentTime < HEAT_MPU_SECONDS) {
    getAngles(false);
  }
  #ifdef DEBUG_IMU
    Serial.println(F("Calculating offsets for mpu6050..."));
  #endif
  currentTime = millis();
  int reads = 0;
  while (millis() - currentTime < OFFSETS_MPU_SECONDS) {
    getAngles(false);
    offsetPitch -= anglePitch;
    offsetRoll -= angleRoll;
    reads++;
  }
  offsetPitch /= (float) reads;
  offsetRoll /= (float) reads;
  #ifdef DEBUG_IMU
    Serial.print(F("offsetPitch: "));
    Serial.print(offsetPitch);
    Serial.print(F("\toffsetRoll: "));
    Serial.println(offsetRoll);
  #endif
}

void initMPU6050() {
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
  mpu.setXGyroOffset(58);
  mpu.setYGyroOffset(31);
  mpu.setZGyroOffset(-25);
  mpu.setZAccelOffset(1770); // 1688 factory default for my test chip

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
      attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
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

  #ifdef DEBUG_IMU
    Serial.println(F("MPU6050 initialized"));
  #endif
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
    delay(500);
  #endif

  // PID
  pidPitch.SetOutputLimits(PITCH_PID_MIN, PITCH_PID_MAX);
  pidPitch.SetMode(AUTOMATIC);
  pidPitch.SetSampleTime(10);
  pidRoll.SetOutputLimits(ROLL_PID_MIN, ROLL_PID_MAX);
  pidRoll.SetMode(AUTOMATIC);
  pidRoll.SetSampleTime(10);

  // RADIO
  radio = new RF24(NFR24L01_CE, NFR24L01_CSN);

  #ifdef NORMAL_MODE
    throttle = ZERO_VALUE_MOTOR;
    cm = CONTROL_MODE_ACRO;
  #endif

  #ifdef CALIBRATION_MODE
    throttle = 1200;
    cm = CONTROL_MODE_ACRO;
  #endif

  radio->begin();
  radio->setDataRate(RF24_250KBPS);
  radio->setPALevel(RF24_PA_MAX); // RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
  radio->openReadingPipe(1, radioAddress);
  radio->startListening();

  // HC-SR04
  //pinMode(HCSR04_ECHO_PIN, INPUT);
  //pinMode(HCSR04_TRIGGER_PIN, OUTPUT);

}

void countDown() {
  #ifdef DEBUG
    Serial.print(F("Countdown: "));
  #endif
  for (int i = SECONDS_COUNTDOWN; i >= 1; i--) {
    delay(500);
    disableLED();
    #ifdef DEBUG
      Serial.print(i);
      if (i > 1) {
        Serial.print(F(","));
      }
      else {
        Serial.print(F("..."));
      }
    #endif
    delay(500);
    enableLED();
  }
  #ifdef DEBUG
    Serial.println(F("0!"));
    Serial.println(F("Quadcopter initialized"));
  #endif
}

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  #ifdef DEBUG
    Serial.begin(38400);
    while (!Serial) {}
  #endif

  enableLED();
  initVars();
  connectMotors();
  armMotors();
  initMPU6050();
  calculateOffsets();
  countDown();
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

  // Calculate velocities of each motor depending of ControlMode
  calculateVelocities();

  // Send velocities to motors
  updateMotorsVelocities();

  printFrequency();

  prev_time = micros();
}
