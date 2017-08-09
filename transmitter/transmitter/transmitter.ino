#include "Controller.h"

Controller *c;

void setup() {
  #ifdef DEBUG
    Serial.begin(38400);
    while (!Serial){}
    delay(1000);
  #endif

  c = new Controller();

  #ifdef NORMAL_MODE
    c->calibrateJoysticks();
  #endif
}

void loop() {

  #ifdef NORMAL_MODE
    c->getControllerData();
    c->calculateSetpoints();
    c->sendRadioInfo();
  #endif

  #ifdef CALIBRATION_MODE
    c->readPotentiometers();
    c->sendCalibrationData();
  #endif

	delay(25);
}
