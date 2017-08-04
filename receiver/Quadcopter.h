#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <Servo.h>
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

// MOTORS
#define PIN_MOTOR_FL 3
#define PIN_MOTOR_FR 5
#define PIN_MOTOR_BL 6
#define PIN_MOTOR_BR 9
#define ARM_MOTOR 1000
#define MIN_VALUE_MOTOR 1100
#define MAX_VALUE_MOTOR 2000
#define MIN_VALUE_PID -1000.0
#define MAX_VALUE_PID 1000.0

// RADIO
#define RADIO_ADDRESS 0xABCDABCD71LL
#define NFR24L01_CE 8
#define NFR24L01_CSN 10

// HC-SR04
#define HCSR04_ECHO_PIN 2
#define HCSR04_TRIGGER_PIN 4

// IMU
#define MIN_PITCH -30
#define MAX_PITCH 30
#define MIN_ROLL -30
#define MAX_ROLL 30
#define MIN_YAW -45
#define MAX_YAW 45

class Quadcopter {

	struct SetPoints {
		int pitch;
		float roll;
		float yaw;
		float throttle;
	};

	private:
		// BMP180
		Adafruit_BMP085 bmp;

		// PID's
		double kpPitch = 1, kiPitch = 0, kdPitch = 0;
		double kpRoll = 1, kiRoll = 0, kdRoll = 0;
		double kpYaw = 0, kiYaw = 0, kdYaw = 0;
		PID *pidRoll, *pidPitch, *pidYaw;

		// MOTORS
		int vFL, vFR, vBL, vBR;
		Servo motorFL, motorFR, motorBL, motorBR;

		// RADIO
		int throttle;
		float desiredPitch, desiredRoll, desiredYaw;
		RF24 &radio;

		// IMU
    MPU9250 myIMU;
		float currentPitch, currentRoll, currentYaw, temperatureIMU;

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
