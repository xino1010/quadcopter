#include "Controller.h"

Controller *c;

void setup() {
  #ifdef DEBUG
    Serial.begin(38400);
    while (!Serial){}
    delay(1000);
  #endif

  c = new Controller();
}

void loop() {

  #ifdef NORMAL_MODE
    c->getControllerData();
    c->sendRadioInfo();
    c->showAngles();
  #endif

  #ifdef CALIBRATION_MODE
    c->readPotentiometers();
    c->readResetButton();
    c->sendCalibrationData();
  #endif

	delay(25);
}
