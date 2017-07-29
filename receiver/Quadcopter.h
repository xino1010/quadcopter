#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <Servo.h>
#include "RF24.h"
#include "PID.h"

#define DEBUG 1

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

class Quadcopter {
	
	struct SetPoints {
		int pitch;
		int roll;
		int yaw;
		int throttle;
	};

	private:
		// BMP180
		Adafruit_BMP085 bmp;

		// PID's
		double kpPitch = 1, kiPitch = 0, kdPitch = 0;
		double kpRoll = 1, kiRoll = 0, kdRoll = 0;
		double kpYaw = 0, kiYaw = 0, kdYaw = 0;
		PID &pidRoll, &pidPitch, &pidYaw;

		// MOTORS
		int vFL, vFR, vBL, vBR;
		Servo motorFL, motorFR, motorBL, motorBR;

		// RADIO
		int desiredPitch, desiredRoll, desiredYaw, throttle;
		RF24 &radio;

		// IMU
		int currentPitch, currentRoll, currentYaw;

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
		void setDesiredPitch(int desiredPitch);
		int getDesidedRoll();
		void setDesiredRoll(int desiredRoll);
		int getDesiredYaw();
		void setDesiredYaw(int desiredYaw);
		int getThrottle();
		void setThrottle(int throttle);
		void updateRadioInfo();

		// IMU
		int getCurrentPitch();
		void setCurrentPitch(int currentPitch);
		int getCurrentRoll();
		void setCurrentRoll(int currentRoll);
		int getCurrentYaw();
		void setCurrentYaw(int currentYaw);

		// HC-SR04
		int getDistance();

};
