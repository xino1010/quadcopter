#define DEBUG
#define JOYSTICK1
#define JOYSTICK2
#define MIN_ANALOG_VALUE 0
#define MEDIUM_ANALOG_VALUE 512
#define MAX_ANALOG_VALUE 1023
#define MIN_THROTTLE 1100
#define MAX_THROTTLE 2000
#define MIN_PITCH -45
#define MAX_PITCH 45
#define MIN_ROLL -45
#define MAX_ROLL 45
#define MIN_YAW -30
#define MAX_YAW 30

struct JoystickInfo {
  int num;
  int pinX;
  int pinY;
  int valX;
  int valY;
};

JoystickInfo j1 = {1, A0, A1, MEDIUM_ANALOG_VALUE, 0};
JoystickInfo j2 = {2, A2, A3, MEDIUM_ANALOG_VALUE, MEDIUM_ANALOG_VALUE};

struct SetPoints {
  float throttle;
  double pitch;
  double roll;
  double yaw;
};

SetPoints sp = {1000, 0.0, 0.0, 0.0};

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
  sp.throttle = constrain(map(j1.valY, MEDIUM_ANALOG_VALUE, MAX_ANALOG_VALUE, MIN_THROTTLE, MAX_THROTTLE), MIN_THROTTLE, MAX_THROTTLE);
  sp.pitch = constrain(map(j2.valY, MIN_ANALOG_VALUE, MAX_ANALOG_VALUE, MIN_PITCH, MAX_PITCH), MIN_PITCH, MAX_PITCH);
  sp.roll = constrain(map(j2.valX, MIN_ANALOG_VALUE, MAX_ANALOG_VALUE, MIN_ROLL, MAX_ROLL), MIN_ROLL, MAX_ROLL);
  sp.yaw = constrain(map(j1.valX, MIN_ANALOG_VALUE, MAX_ANALOG_VALUE, MIN_YAW, MAX_YAW), MIN_YAW, MAX_YAW);
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
}

void loop() {
  #ifdef JOYSTICK1
    readJoystick1();
  #endif
  #ifdef JOYSTICK2
    readJoystick2();
  #endif
  calculateSetpoints();
  #ifdef DEBUG
    #ifdef JOYSTICK1
      printJoystickData(&j1);
    #endif
    #ifdef JOYSTICK2
      printJoystickData(&j2);
    #endif
    printSetpoints();
  #endif
	delay(100);
}
