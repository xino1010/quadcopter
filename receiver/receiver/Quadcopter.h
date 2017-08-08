#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <Servo.h>
#include <nRF24L01.h>
#include "RF24.h"
#include "PID.h"
#include "quaternionFilters.h"
#include "MPU9250.h"

#define DEBUG
#ifdef DEBUG
  //#define DEBUG_IMU
  #define DEBUG_MOTORS
#endif

// CONSTANTS

// BMP180
#define MIN_ALTITUDE 3
#define MAX_ALTITUDE 50
#define TIME_READ_ALTITUDE 3000

// MOTORS
#define PIN_MOTOR_FL 3
#define PIN_MOTOR_FR 5
#define PIN_MOTOR_BL 6
#define PIN_MOTOR_BR 9
#define ARM_MOTOR 900
#define ZERO_VALUE_MOTOR 1000
#define MIN_VALUE_MOTOR 1300
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
#define MAX_CALIBRATION_ATTEMPTS 10
#define NUMBER_OF_READINGS_IMU 150
#define OFFSET_ANGLE 0.5

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
		double kpPitch = 1, kiPitch = 0, kdPitch = 0;
		double kpRoll = 1, kiRoll = 0, kdRoll = 0;
		double kpYaw = 0, kiYaw = 0, kdYaw = 0;
		double kpDistance = 0, kiDistance = 0, kdDistance = 0;
		double kpAltitude = 0, kiAltitude = 0, kdAltitude = 0;
		PID *pidRoll, *pidPitch, *pidYaw, *pidDistance, *pidAltitude;

		// MOTORS
		int vFL, vFR, vBL, vBR;
		Servo motorFL, motorFR, motorBL, motorBR;
    void connectMotors();
    void armMotors();
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
    float radioData[7];
    int cm;
    bool controlModeChange;
    int getDesiredPitch();
    void setDesiredPitch(float desiredPitch);
    int getDesidedRoll();
    void setDesiredRoll(float desiredRoll);
    int getDesiredYaw();
    void setDesiredYaw(float desiredYaw);
    int getThrottle();
    void setThrottle(int throttle);
    void setControlMode(int cm);

		// IMU
    MPU9250 myIMU;
		float currentPitch, currentRoll, currentYaw, temperatureIMU;
    int getCurrentPitch();
    void setCurrentPitch(float currentPitch);
    int getCurrentRoll();
    void setCurrentRoll(float currentRoll);
    int getCurrentYaw();
    void setCurrentYaw(float currentYaw);
    void getReadingsIMU(float *avgAngles);
    float getTemperatureIMU();
    void setTemperatureIMU(float temperatureIMU);

    // HC-SR04
    int getDistance();

	public:
		Quadcopter();

    // BMP180

		// PID's

		// MOTORS
    void stopMotors();
    void calculateVelocities();
    void updateMotorsVelocities();

		// RADIO
    void updateRadioInfo();
    int getControlMode();

		// IMU
    void initIMU();
    bool isCalibrated();
    void calibrateIMU();
    void updateAngles();

    // HC-SR04


};
