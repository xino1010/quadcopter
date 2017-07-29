#include <SPI.h>
#include "RF24.h"

#define DEBUG
#define MIN_ANALOG_VALUE 0
#define MEDIUM_ANALOG_VALUE 512
#define MAX_ANALOG_VALUE 1023
#define MIN_THROTTLE 1100
#define MAX_THROTTLE 2000
#define MIN_PITCH 1000
#define MEDIUM_PITCH 1500
#define MAX_PITCH 2000
#define MIN_ROLL 1000
#define MEDIUM_ROLL 1500
#define MAX_ROLL 2000
#define MIN_YAW 1000
#define MEDIUM_YAW 1500
#define MAX_YAW 2000
#define THRESHOLD 20

#define RADIO_ADDRESS 0xABCDABCD71LL
#define NFR24L01_CE 9
#define NFR24L01_CSN 10

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
  int pitch;
  int roll;
  int yaw;
};

SetPoints sp = {MIN_THROTTLE, MEDIUM_PITCH, MEDIUM_ROLL, MEDIUM_YAW};
const int sizeOfSetPoints = sizeof(sp);

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
  sp.throttle = (int) constrain(map(j1.valY, MEDIUM_ANALOG_VALUE, MAX_ANALOG_VALUE, MIN_THROTTLE, MAX_THROTTLE), MIN_THROTTLE, MAX_THROTTLE);
  // PITCH
  sp.pitch = (int) constrain(map(j2.valY, MIN_ANALOG_VALUE, MAX_ANALOG_VALUE, MIN_PITCH, MAX_PITCH), MIN_PITCH, MAX_PITCH);
  if (sp.pitch >= MEDIUM_PITCH - THRESHOLD && sp.pitch <= MEDIUM_PITCH + THRESHOLD) {
    sp.pitch = MEDIUM_PITCH;
  }
  // ROLL
  sp.roll = (int) constrain(map(j2.valX, MIN_ANALOG_VALUE, MAX_ANALOG_VALUE, MIN_ROLL, MAX_ROLL), MIN_ROLL, MAX_ROLL);
  if (sp.roll >= MEDIUM_ROLL - THRESHOLD && sp.roll <= MEDIUM_ROLL + THRESHOLD) {
    sp.roll = MEDIUM_ROLL;
  }
  // YAW
  sp.yaw = (int) constrain(map(j1.valX, MIN_ANALOG_VALUE, MAX_ANALOG_VALUE, MIN_YAW, MAX_YAW), MIN_YAW, MAX_YAW);
  if (sp.yaw >= MEDIUM_YAW - THRESHOLD && sp.yaw <= MEDIUM_YAW + THRESHOLD) {
    sp.yaw = MEDIUM_YAW;
  }
}

void printSetpoints() {
  Serial.print("Throttle: ");
  Serial.print(sp.throttle);
  Serial.print(" Pitch: ");
  Serial.print(sp.pitch);
  Serial.print(" Roll: ");
  Serial.print(sp.roll);
  Serial.print("º Yaw: ");
  Serial.print(sp.yaw);
  Serial.print("º");
  Serial.println();
}

void setup() {
	Serial.begin(9600);
  radio.begin();
  radio.setPALevel(RF24_PA_HIGH); // RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
  radio.openWritingPipe(RADIO_ADDRESS);
  radio.stopListening();
}

void loop() {
  readJoystick1();
  readJoystick2();
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

	delay(20);
}
