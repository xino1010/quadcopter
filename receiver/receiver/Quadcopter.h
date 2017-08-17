#ifndef _QUADCOPTER_H_
#define _QUADCOPTER_H_

#include <Arduino.h>
#include <Adafruit_BMP085.h>
#include <Servo.h>
#include <nRF24L01.h>
#include "RF24.h"
#include "PID.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

#define DEBUG
#ifdef DEBUG
  //#define DEBUG_BMP
  #define DEBUG_IMU
  #define DEBUG_MOTORS
  //#define DEBUG_RADIO
  //#define DEBUG_SONAR
  //#define DEBUG_PID
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
#define MIN_ALTITUDE 300
#define MAX_ALTITUDE 5000
#define TIME_READ_ALTITUDE 3000
#define DESIRED_ALTITUDE 3000

// MOTORS
#define PIN_MOTOR_FL 3
#define PIN_MOTOR_FR 5
#define PIN_MOTOR_BL 6
#define PIN_MOTOR_BR 9
#define ARM_MOTOR 800
#define ZERO_VALUE_MOTOR 900
#define MIN_VALUE_MOTOR 1100
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
#define HCSR04_ECHO_PIN 2
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
#define MIN_YAW -45
#define MAX_YAW 45
#define OFFSET_ANGLE 0.5
#define INTERRUPT_PIN 2
#define NORMALIZE_SAMPLES 1250
#define OFFSET_SAMPLES 500

// LED
#define LED_PIN 7

class Quadcopter {

	private:

    int myAbs(int value);

		// BMP180
		Adafruit_BMP085 bmp;
    unsigned long previousAltitudeRead;
    float offsetAltitude;
    float getAltitude();
    float getTemperature();

		// PID's
		double kpPitch = 0, kiPitch = 0, kdPitch = 0;
		double kpRoll = 1, kiRoll = 0, kdRoll = 0;
		double kpYaw = 0, kiYaw = 0, kdYaw = 0;
		double kpDistance = 0, kiDistance = 0, kdDistance = 0;
		double kpAltitude = 0, kiAltitude = 0, kdAltitude = 0;
		PID *pidRoll, *pidPitch, *pidYaw, *pidDistance, *pidAltitude;

		// MOTORS
		int vFL, vFR, vBL, vBR;
		Servo motorFL, motorFR, motorBL, motorBR;
    int getVelocityFL();
    void setVelocityFL(int vFL);
    int getVelocityFR();
    void setVelocityFR(int vFR);
    int getVelocityBL();
    void setVelocityBL(int vBL);
    int getVelocityBR();
    void setVelocityBR(int vBR);

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
    bool controlModeChange;
    int getDesiredPitch();
    void setDesiredPitch(float desiredPitch);
    int getDesiredRoll();
    void setDesiredRoll(float desiredRoll);
    int getDesiredYaw();
    void setDesiredYaw(float desiredYaw);
    int getThrottle();
    void setThrottle(int throttle);
    void setControlMode(int cm);

		// IMU
    MPU6050 mpu;
    // MPU control/status vars
    bool dmpReady = false;  // set true if DMP init was successful
    uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
    uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer

    // orientation/motion vars
    Quaternion q;           // [w, x, y, z]         quaternion container
    VectorInt16 aa;         // [x, y, z]            accel sensor measurements
    VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
    VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
    VectorFloat gravity;    // [x, y, z]            gravity vector
    float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

    float currentPitch, currentRoll, currentYaw, temperatureIMU;
    float offsetPitch, offsetRoll, offsetYaw;
    float getCurrentPitch();
    void setCurrentPitch(float currentPitch);
    float getCurrentRoll();
    void setCurrentRoll(float currentRoll);
    float getCurrentYaw();
    void setCurrentYaw(float currentYaw);

    volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

    // HC-SR04
    int getDistance();

	public:
		Quadcopter();

    // BMP180

		// PID's

		// MOTORS
    void armMotors();
    void connectMotors();
    void stopMotors();
    void calculateVelocities();
    void updateMotorsVelocities();

		// RADIO
    void updatePIDInfo();
    void updateRadioInfo();
    int getControlMode();

		// IMU
    void calculateOffsets();
    void initIMU();
    void updateAngles();
    bool getDmpReady();
    void dmpDataReady();

    // HC-SR04

    // LED
    void enableLED();
    void disableLED();

};

#endif
