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
  
  #ifdef TRANSMITTER
    c->getControllerData();
    c->calculateSetpoints();
    c->sendRadioInfo();
  #endif

  #ifdef CALIBRATION
    c->readPotentiometers();
    c->sendCalibrationData();
    c->sendCalibrationData();
  #endif

	delay(50);
}
