#include <Adafruit_BMP085.h>
#include <Servo.h>
#include <nRF24L01.h>
#include "RF24.h"
#include <Wire.h>
#include <Kalman.h>
#include <PID_v1.h>

#define DEBUG
#ifdef DEBUG
  //#define DEBUG_BMP
  //#define DEBUG_IMU
  #define DEBUG_MOTORS
  //#define DEBUG_RADIO
  //#define DEBUG_PID
  //#define DEBUG_SONAR
  //#define DEBUG_KALMAN
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
#define ZERO_VALUE_MOTOR 1000
#define MIN_VALUE_MOTOR 1100
#define MAX_VALUE_MOTOR 1800

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
#define HEAT_OFFSETS 1000
#define READS_OFFSETS 500
#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead

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
float KP_PITCH = 1;
float KI_PITCH = 0;
float KD_PITCH = 0;
double pidPitchIn, pidPitchOut, pidPitchSetpoint = 0;
PID pidPitch(&pidPitchIn, &pidPitchOut, &pidPitchSetpoint, KP_PITCH, KI_PITCH, KD_PITCH, DIRECT);

// Roll
#define ROLL_PID_MIN -45 
#define ROLL_PID_MAX 45
float KP_ROLL = 0;
float KI_ROLL = 0;
float KD_ROLL = 0;
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
const uint8_t IMUAddress = 0x68; // AD0 is logic low on the PCB
const uint16_t I2C_TIMEOUT = 1000; // Used to check for errors in I2C communication
float offsetPitch, offsetRoll, offsetYaw = 0;
float anglePitch, angleRoll, angleYaw = 0;
Kalman kalmanX;
Kalman kalmanY;
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;
double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

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
      velocityFL = throttle - pidPitchOut + pidRollOut;
      velocityFR = throttle - pidPitchOut - pidRollOut;
      velocityBL = throttle + pidPitchOut + pidRollOut;
      velocityBR = throttle + pidPitchOut - pidRollOut;

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
      Serial.print(F("\tPitch: "));
      Serial.print(pidPitchSetpoint);
      Serial.print(F("\tRoll: "));
      Serial.print(pidRollSetpoint);
      //Serial.print(F("\tYaw: "));
      //Serial.print(pidYawSetpoint);
      Serial.print(F("\tCM: "));
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
        Serial.print(F("\tkI: "));
        Serial.print(pidPitch.GetKi());
        Serial.print(F("\tkD: "));
        Serial.println(pidPitch.GetKd());
      #endif
    #endif

    #ifdef CALIBRATION_ROLL
      pidRoll.SetTunings(radioPIDdata[0], radioPIDdata[1], radioPIDdata[2]);
      #ifdef DEBUG_PID
        Serial.print(F("ROLL - kP: "));
        Serial.print(pidRoll.GetKp());
        Serial.print(F("\tkI: "));
        Serial.print(pidRoll.GetKi());
        Serial.print(F("\tkD: "));
        Serial.println(pidRoll.GetKd());
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
uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop) {
  return i2cWrite(registerAddress, &data, 1, sendStop); // Returns 0 on success
}

uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) {
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data, length);
  uint8_t rcode = Wire.endTransmission(sendStop); // Returns 0 on success
  if (rcode) {
    Serial.print(F("i2cWrite failed: "));
    Serial.println(rcode);
  }
  return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
}

uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes) {
  uint32_t timeOutTimer;
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  uint8_t rcode = Wire.endTransmission(false); // Don't release the bus
  if (rcode) {
    Serial.print(F("i2cRead failed: "));
    Serial.println(rcode);
    return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
  }
  Wire.requestFrom(IMUAddress, nbytes, (uint8_t)true); // Send a repeated start and then release the bus after reading
  for (uint8_t i = 0; i < nbytes; i++) {
    if (Wire.available())
      data[i] = Wire.read();
    else {
      timeOutTimer = micros();
      while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
      if (Wire.available())
        data[i] = Wire.read();
      else {
        Serial.println(F("i2cRead timeout"));
        return 5; // This error value is not already taken by endTransmission
      }
    }
  }
  return 0; // Success
}

// IMU
void getAngles(bool useOffsets) {

  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
  #ifdef RESTRICT_PITCH // Eq. 25 and 26
    double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  #else // Eq. 28 and 29
    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  #endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

  #ifdef RESTRICT_PITCH
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
      kalmanX.setAngle(roll);
      kalAngleX = roll;
      gyroXangle = roll;
    }
    else {
      kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
    }

    if (abs(kalAngleX) > 90) {
      gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
    }
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
  #else
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
      kalmanY.setAngle(pitch);
      compAngleY = pitch;
      kalAngleY = pitch;
      gyroYangle = pitch;
    }
    else {
      kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter
    }

    if (abs(kalAngleY) > 90) {
      gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
    }
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
  #endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180) {
    gyroXangle = kalAngleX;
  }
  if (gyroYangle < -180 || gyroYangle > 180) {
    gyroYangle = kalAngleY;
  }

  anglePitch = kalAngleY; // Pitch
  angleRoll = kalAngleX; // Roll

  if (useOffsets) {
    anglePitch += offsetPitch;
    angleRoll += offsetRoll;
  }

  #ifdef DEBUG_IMU
    Serial.print(F("Pitch: "));
    Serial.print(anglePitch);
    Serial.print(F("\tRoll: "));
    Serial.print(angleRoll);
    Serial.print(F("\tYaw: "));
    Serial.println(angleYaw);
  #endif
}

void calculateOffsets() {
  #ifdef DEBUG_IMU
    Serial.println(F("Heating mpu6050..."));
  #endif
  for (int i = 0; i < HEAT_OFFSETS; i++) {
    getAngles(false);
  }
  #ifdef DEBUG_IMU
    Serial.println(F("Calculating offsets for mpu6050..."));
  #endif
  for (int i = 0; i < READS_OFFSETS; i++) {
    getAngles(false);
    offsetPitch -= anglePitch;
    offsetRoll -= angleRoll;
  }
  offsetPitch /= (float) READS_OFFSETS;
  offsetRoll /= (float) READS_OFFSETS;
  #ifdef DEBUG_IMU
    Serial.print(F("offsetPitch: "));
    Serial.print(offsetPitch);
    Serial.print(F("\toffsetRoll: "));
    Serial.println(offsetRoll);
  #endif
}

void initMPU6050() {
  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    #ifdef DEBUG_IMU
      Serial.print(F("Error reading sensor"));
    #endif
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
  #ifdef RESTRICT_PITCH // Eq. 25 and 26
    double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  #else // Eq. 28 and 29
    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  #endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  timer = micros();
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
  pidPitch.SetSampleTime(5);
  pidRoll.SetOutputLimits(ROLL_PID_MIN, ROLL_PID_MAX);
  pidRoll.SetMode(AUTOMATIC);
  pidRoll.SetSampleTime(5);

  // RADIO
  radio = new RF24(NFR24L01_CE, NFR24L01_CSN);

  #ifdef NORMAL_MODE
    throttle = ZERO_VALUE_MOTOR;
    cm = CONTROL_MODE_ACRO;
  #endif

  #ifdef CALIBRATION_MODE
    throttle = 1250;
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
  Wire.begin();
  #if ARDUINO >= 157
    Wire.setClock(400000UL); // Set I2C frequency to 400kHz
  #else
    TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
  #endif

  #ifdef DEBUG
    Serial.begin(38400);
    while (!Serial) {}
  #endif

  enableLED();
  initVars();
  initMPU6050();
  calculateOffsets();
  connectMotors();
  armMotors();
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
