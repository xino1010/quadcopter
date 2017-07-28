#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <Servo.h>
#include "PID.h"

#define DEBUG 1

// PINS
#define PIN_MOTOR_FL 3
#define PIN_MOTOR_FR 5
#define PIN_MOTOR_BL 6
#define PIN_MOTOR_BR 9

// CONSTANTS
#define ARM_MOTOR 1000
#define MIN_VALUE_MOTOR 1100
#define MAX_VALUE_MOTOR 2000
#define MIN_VALUE_PID -1000.0
#define MAX_VALUE_PID 1000.0

class Quadcopter {
	
	struct JoystickInfo {
		float pitch;
		float roll;
		float yaw;
		float throttle;
	};

	private:
		// BMP180
		Adafruit_BMP085 bmp;

		// PID's
		// Pitch
		double kpPitch = 1;
		double kiPitch = 0;
		double kdPitch = 0;
		PID &pidPitch;
		// Roll
		double kpRoll = 1;
		double kiRoll = 0;
		double kdRoll = 0;
		PID &pidRoll;
		// Yaw
		double kpYaw = 0;
		double kiYaw = 0;
		double kdYaw = 0;
		PID &pidYaw;

		// MOTORS
		float vFL, vFR, vBL, vBR;
		Servo motorFL;
		Servo motorFR;
		Servo motorBL;
		Servo motorBR;

		// RADIO
		float desiredPitch;
		float desiredRoll;
		float desiredYaw;
		float throttle;

		// IMU
		float currentPitch;
		float currentRoll;
		float currentYaw;

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
		float getVelocityFL();
		void setVelocityFL(float vFL);
		float getVelocityFR();
		void setVelocityFR(float vFR);
		float getVelocityBL();
		void setVelocityBL(float vBL);
		float getVelocityBR();
		void setVelocityBR(float vBR);

		// RADIO
		float getDesiredPitch();
		void setDesiredPitch(float desiredPitch);
		float getDesidedRoll();
		void setDesiredRoll(float desiredRoll);
		float getDesiredYaw();
		void setDesiredYaw(float desiredYaw);
		float getThrottle();
		void setThrottle(float throttle);
		void updateRadioInfo();

		// IMU
		float getCurrentPitch();
		void setCurrentPitch(float currentPitch);
		float getCurrentRoll();
		void setCurrentRoll(float currentRoll);
		float getCurrentYaw();
		void setCurrentYaw(float currentYaw);

};
