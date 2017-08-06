#include <SPI.h>
#include "RF24.h"

#define DEBUG
//#define TRANSMITTER
#define CALIBRATION

// TRANSMITTER
#define MIN_ANALOG_VALUE 0
#define MEDIUM_ANALOG_VALUE 512
#define MAX_ANALOG_VALUE 1023
#define ZERO_THROTTLE 1000
#define MIN_THROTTLE 1300
#define MAX_THROTTLE 2000
#define MIN_PITCH -30
#define MEDIUM_PITCH 0
#define MAX_PITCH 30
#define MIN_ROLL -30
#define MEDIUM_ROLL 0
#define MAX_ROLL 30
#define MIN_YAW -45
#define MEDIUM_YAW 0
#define MAX_YAW 45
#define THRESHOLD 2

// RADIO
#define RADIO_ADDRESS 0xABCDABCD71LL
#define NFR24L01_CE 9
#define NFR24L01_CSN 10

// CALIBRATION
#define POTENTIOMETRE_KP A1
#define POTENTIOMETRE_KI A2
#define POTENTIOMETRE_KD A3
#define MIN_KP 0.0
#define MAX_KP 4.0
#define MIN_KI 0.0
#define MAX_KI 1.0
#define MIN_KD 0.0
#define MAX_KD 400.0

// CONTROL
#define LED_STATUS 2
#define LED_HOLD_POSITION 4
#define BUTTON_STATUS 7
#define BUTTON_HOLD_POSITION 8

struct JoystickInfo {
  int num;
  int pinX;
  int pinY;
  int valX;
  int valY;
};

JoystickInfo j1 = {1, A0, A1, MEDIUM_ANALOG_VALUE, 0};
JoystickInfo j2 = {2, A2, A3, MEDIUM_ANALOG_VALUE, MEDIUM_ANALOG_VALUE};

RF24 radio(NFR24L01_CE, NFR24L01_CSN);

struct SetPoints {
  int throttle;
  float pitch;
  float roll;
  float yaw;
  int status;
  int holdPosition;
};

SetPoints sp = {MIN_THROTTLE, MEDIUM_PITCH, MEDIUM_ROLL, MEDIUM_YAW, LOW, LOW};
const int sizeOfSetPoints = sizeof(sp);

struct CalibrationData {
  float kP;
  float kI;
  float kD;
};

CalibrationData cd = {MIN_KP, MIN_KI, MIN_KD};
const int sizeOfCalibration = sizeof(cd);

void readJoystick1() {
  j1.valX = analogRead(j1.pinX);
  j1.valY = MAX_ANALOG_VALUE - analogRead(j1.pinY);
}

void readJoystick2() {
  j2.valX = analogRead(j2.pinX);
  j2.valY = analogRead(j2.pinY);
}

void printJoystickData(struct JoystickInfo *ji) {
  Serial.print("JOYSTICK ");
  Serial.print(ji->num);
  Serial.print(" -> X: ");
  Serial.print(ji->valX);
  Serial.print(" Y: ");
  Serial.println(ji->valY);
}

void calculateSetpoints() {
  // THROTTLE
  sp.throttle = map(j1.valY, MEDIUM_ANALOG_VALUE, MAX_ANALOG_VALUE, MIN_THROTTLE, MAX_THROTTLE);
  // PITCH
  sp.pitch = map(j2.valY, MIN_ANALOG_VALUE, MAX_ANALOG_VALUE, MIN_PITCH, MAX_PITCH);
  if (sp.pitch >= MEDIUM_PITCH - THRESHOLD && sp.pitch <= MEDIUM_PITCH + THRESHOLD) {
    sp.pitch = MEDIUM_PITCH;
  }
  // ROLL
  sp.roll = map(j2.valX, MIN_ANALOG_VALUE, MAX_ANALOG_VALUE, MIN_ROLL, MAX_ROLL);
  if (sp.roll >= MEDIUM_ROLL - THRESHOLD && sp.roll <= MEDIUM_ROLL + THRESHOLD) {
    sp.roll = MEDIUM_ROLL;
  }
  // YAW
  sp.yaw = map(j1.valX, MIN_ANALOG_VALUE, MAX_ANALOG_VALUE, MIN_YAW, MAX_YAW);
  if (sp.yaw >= MEDIUM_YAW - THRESHOLD && sp.yaw <= MEDIUM_YAW + THRESHOLD) {
    sp.yaw = MEDIUM_YAW;
  }
}

void printSetpoints() {
  Serial.print("Throttle: ");
  Serial.print(sp.throttle);
  Serial.print("º \tPitch: ");
  Serial.print(sp.pitch);
  Serial.print("º \tRoll: ");
  Serial.print(sp.roll);
  Serial.print("º \tYaw: ");
  Serial.print(sp.yaw);
  Serial.print("º \tStatus");
  Serial.print(sp.status);
  Serial.print(" \tHoldPos: ");
  Serial.print(sp.holdPosition);
  Serial.println();
}

float mapfloat(long x, long in_min, long in_max, long out_min, long out_max) {
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

void readPotentiometers() {
  cd.kP = analogRead(POTENTIOMETRE_KP);
  cd.kI = analogRead(POTENTIOMETRE_KI);
  cd.kD = analogRead(POTENTIOMETRE_KD);
  #ifdef DEBUG
    Serial.print("AkP: ");
    Serial.print(cd.kP);
    Serial.print("\tAkI: ");
    Serial.print(cd.kI);
    Serial.print("\tAkD: ");
    Serial.print(cd.kD);
  #endif

  cd.kP = mapfloat(cd.kP, MIN_ANALOG_VALUE, MAX_ANALOG_VALUE, MIN_KP, MAX_KP);
  cd.kI = mapfloat(cd.kI, MIN_ANALOG_VALUE, MAX_ANALOG_VALUE, MIN_KI, MAX_KI);
  cd.kD = mapfloat(cd.kD, MIN_ANALOG_VALUE, MAX_ANALOG_VALUE, MIN_KD, MAX_KD);

  #ifdef DEBUG
    Serial.print("\tkP: ");
    Serial.print(cd.kP);
    Serial.print("\tkI: ");
    Serial.print(cd.kI);
    Serial.print("\tkD: ");
    Serial.println(cd.kD);
  #endif

}

void readButtons() {
  sp.status = digitalRead(BUTTON_STATUS);
  sp.holdPosition = digitalRead(BUTTON_HOLD_POSITION);
}

void updateLeds() {
  digitalWrite(LED_STATUS, sp.status);
  digitalWrite(LED_HOLD_POSITION, sp.holdPosition);
}

void setup() {
	Serial.begin(9600);

  // CONTORL
  pinMode(LED_STATUS, OUTPUT);
  pinMode(LED_HOLD_POSITION, OUTPUT);
  pinMode(BUTTON_STATUS, INPUT);
  pinMode(BUTTON_HOLD_POSITION, INPUT);

  radio.begin();
  radio.setPALevel(RF24_PA_HIGH); // RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
  radio.openWritingPipe(RADIO_ADDRESS);
  radio.stopListening();
}

void loop() {
  #ifdef TRANSMITTER
    readJoystick1();
    readJoystick2();
    readButtons();
    calculateSetpoints();
    #ifdef DEBUG
      printJoystickData(&j1);
      printJoystickData(&j2);
      printSetpoints();
    #endif

    if (!radio.write(&sp, sizeOfSetPoints)) {
      #ifdef DEBUG
        Serial.println("Failed transmitting setpoints");
      #endif
    }

    updateLeds();
  #endif

  #ifdef CALIBRATION
    readPotentiometers();
    /*if (!radio.write(&cd, sizeOfCalibration)) {
      #ifdef DEBUG
        Serial.println("Failed transmitting setpoints");
      #endif
    }*/
  #endif

	delay(100);
}
