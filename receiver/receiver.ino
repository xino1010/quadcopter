#include <Servo.h>
#include "PID.h"

// PINS
#define PIN_MOTOR_FL 1
#define PIN_MOTOR_FR 1
#define PIN_MOTOR_BL 1
#define PIN_MOTOR_BR 1

// CONSTANTS
#define RAD_TO_DEG (180 / 3.141592654)
#define ARM_MOTOR 1000
#define MIN_VALUE_MOTOR 1100
#define MAX_VALUE_MOTOR 2000
#define MIN_VALUE_PID -1000
#define MAX_VALUE_PID 1000

float pwdFL, pwdFR, pwdBL, pwdBR;
Servo motorFL;
Servo motorFR;
Servo motorBL;
Servo motorBR;

// PITCH
float kpPitch = 1;
float kiPitch = 0;
float kdPitch = 0;
PID pidPitch(kpPitch, kiPitch, kdPitch, MIN_VALUE_PID, MAX_VALUE_PID);

// ROLL
float kpRoll = 1;
float kiRoll = 0;
float kdRoll = 0;
PID pidRoll(kpRoll, kiRoll, kdRoll, MIN_VALUE_PID, MAX_VALUE_PID);

// YAW
float kpYaw = 0;
float kiYaw = 0;
float kdYaw = 0;
PID pidYaw(kpYaw, kiYaw, kdYaw, MIN_VALUE_PID, MAX_VALUE_PID);

double throttle = 1300;
float desiredAngle = 0;

void setup() {
	// Connect Motors
	motorFL.attach(PIN_MOTOR_FL);
	motorFR.attach(PIN_MOTOR_FR);
	motorBL.attach(PIN_MOTOR_BL);
	motorBR.attach(PIN_MOTOR_BR);
	// Send a minimum signal to prepare the motors
	delay(100);
	motorFL.writeMicroseconds(ARM_MOTOR);
	motorFR.writeMicroseconds(ARM_MOTOR);
	motorBL.writeMicroseconds(ARM_MOTOR);
	motorBR.writeMicroseconds(ARM_MOTOR);
	delay(3000);
}

void loop() {
	pidPitch.setDesiredPoint(0);
	pidPitch.setCurrentPoint(0);
	double resultPitch = pidPitch.calculate();
	pidRoll.setDesiredPoint(0);
	pidRoll.setCurrentPoint(0);
	double resultRoll = pidRoll.calculate();
	pidYaw.setDesiredPoint(0);
	pidYaw.setCurrentPoint(0);
	double resultYaw = pidYaw.calculate();

	// Calculate velocities of each motor
	pwdFL = throttle + resultRoll + resultPitch + resultYaw;
	pwdFR = throttle - resultRoll + resultPitch - resultYaw;
	pwdBL = throttle + resultRoll - resultPitch - resultYaw;
	pwdBR = throttle - resultRoll - resultPitch + resultYaw;
	
	// Check that the speeds do not exceed the limits
	pwdFL = constrain(pwdFL, MIN_VALUE_MOTOR, MAX_VALUE_MOTOR);
	pwdFR = constrain(pwdFR, MIN_VALUE_MOTOR, MAX_VALUE_MOTOR);
	pwdBL = constrain(pwdBL, MIN_VALUE_MOTOR, MAX_VALUE_MOTOR);
	pwdBR = constrain(pwdBR, MIN_VALUE_MOTOR, MAX_VALUE_MOTOR);

	// Send velocities to motors
	motorFL.writeMicroseconds(pwdFL);
	motorFR.writeMicroseconds(pwdFR);
	motorBL.writeMicroseconds(pwdBL);
	motorBR.writeMicroseconds(pwdBR);

	delay(50);
}
