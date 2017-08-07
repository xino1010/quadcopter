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
const byte radioAddress[5] = {'c','a','n','a','l'};
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
#define MIN_PITCH -30
#define MAX_PITCH 30
#define MIN_ROLL -30
#define MAX_ROLL 30
#define MIN_YAW -45
#define MAX_YAW 45

class Quadcopter {

	private:
		// BMP180
		Adafruit_BMP085 bmp;
    float offsetAltitude;

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

		// RADIO
		int throttle;
		float desiredPitch, desiredRoll, desiredYaw;
		RF24 *radio;
    float radioData[6];
    int cm;
    bool controlModeChange;

		// IMU
    MPU9250 myIMU;
		float currentPitch, currentRoll, currentYaw, temperatureIMU;

    // HC-SR04

	public:
		Quadcopter();

		// BMP180
		float getAltitude();
		float getTemperature();

		// PID's

		// MOTORS
		void connectMotors();
		void armMotors();
		void calculateVelocities();
		void updateMotorsVelocities();
		int getVelocityFL();
		void setVelocityFL(int vFL);
		int getVelocityFR();
		void setVelocityFR(int vFR);
		int getVelocityBL();
		void setVelocityBL(int vBL);
		int getVelocityBR();
		void setVelocityBR(int vBR);
    void stopMotors();

		// RADIO
		int getDesiredPitch();
		void setDesiredPitch(float desiredPitch);
		int getDesidedRoll();
		void setDesiredRoll(float desiredRoll);
		int getDesiredYaw();
		void setDesiredYaw(float desiredYaw);
		int getThrottle();
		void setThrottle(int throttle);
		void updateRadioInfo();
    void setControlMode(int cm);
    int getControlMode();

		// IMU
		int getCurrentPitch();
		void setCurrentPitch(float currentPitch);
		int getCurrentRoll();
		void setCurrentRoll(float currentRoll);
		int getCurrentYaw();
		void setCurrentYaw(float currentYaw);
		void initIMU();
		void updateAngles();
    float getTemperatureIMU();
    void setTemperatureIMU(float temperatureIMU);

		// HC-SR04
		int getDistance();
};
