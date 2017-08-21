#include "Controller.h"

Controller::Controller() {
  // TRANSMITTER
  sp.throttle = 0;
  sp.pitch = MEDIUM_ANALOG_VALUE;
  sp.roll = MEDIUM_ANALOG_VALUE;
  sp.yaw = MEDIUM_ANALOG_VALUE;
  sp.status = LOW;
  sp.holdDistance = LOW;
  sp.holdAltitude = LOW;

  // RADIO
  radio = new RF24(NFR24L01_CE, NFR24L01_CSN);
  radio->begin();
  radio->setDataRate(RF24_250KBPS);
  radio->setPALevel(RF24_PA_MAX); // RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
  radio->openWritingPipe(address);
  radio->stopListening();

  // CALIBRATION
  cd.kP = MIN_KP;
  cd.kI = MIN_KI;
  cd.kD = MIN_KD;
  cd.reset =  LOW;
  lastButtonReset = LOW;
  buttonReset = 0;
  pinMode(BUTTON_RESET, INPUT);

  // CONTROL
  pinMode(LED_STATUS, OUTPUT);
  pinMode(LED_HOLD_DISTANCE, OUTPUT);
  pinMode(LED_HOLD_ALTITUDE, OUTPUT);
  pinMode(BUTTON_STATUS, INPUT);
  pinMode(BUTTON_HOLD_DISTANCE, INPUT);
  pinMode(BUTTON_HOLD_ALTITUDE, INPUT);

  j1.pinX = A0;
  j1.pinY = A1;
  j2.pinX = A2;
  j2.pinY = A3;

  lastButtonStatus = LOW;
  lastButtonHoldDistance = LOW;
  lastButtonHoldAltitude = LOW;
  buttonStatus = 0;
  buttonHoldDistance = 0;
  buttonHoldAltitude = 0;

  #ifdef DEBUG
    Serial.println(F("Controller initialized"));
  #endif

}

float Controller::mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

// TRANSMITTER
void Controller::printSetpoints() {
  #ifdef DEBUG_JOYSTICKS
    Serial.print(F("Throttle: "));
    Serial.print(sp.throttle);
    Serial.print(F("\tPitch: "));
    Serial.print(sp.pitch);
    Serial.print(F("\tRoll: "));
    Serial.print(sp.roll);
    Serial.print(F("\tYaw: "));
    Serial.print(sp.yaw);
    Serial.print(F("\tStatus: "));
    Serial.print(sp.status);
    Serial.print(F("\tHoldDist: "));
    Serial.print(sp.holdDistance);
    Serial.print(F("\tHoldAlt: "));
    Serial.println(sp.holdAltitude);
  #endif
}

// RADIO
void Controller::sendRadioInfo() {
  radioData[0] = sp.throttle;
  radioData[1] = sp.pitch;
  radioData[2] = sp.roll;
  radioData[3] = sp.yaw;
  radioData[4] = sp.status;
  radioData[5] = sp.holdDistance;
  radioData[6] = sp.holdAltitude;
  if (radio->write(&radioData, sizeRadioData)) {
    #ifdef DEBUG_RADIO
      Serial.println(F("Data sent"));
    #endif
  }
  else {
    #ifdef DEBUG_RADIO
      Serial.println(F("Failed transmitting setpoints"));
    #endif
  }
}

// CALIBRATION
void Controller::readPotentiometers() {
  cd.kP = analogRead(POTENTIOMETRE_KP);
  cd.kI = analogRead(POTENTIOMETRE_KI);
  cd.kD = analogRead(POTENTIOMETRE_KD);
  #ifdef DEBUG_POTENTIOMETERS
    Serial.print(F("kP: "));
    Serial.print(cd.kP);
    Serial.print(F("\tkI: "));
    Serial.print(cd.kI);
    Serial.print(F("\tkD: "));
    Serial.println(cd.kD);
  #endif

  cd.kP = mapFloat(cd.kP, MAX_ANALOG_VALUE, MIN_ANALOG_VALUE, MIN_KP, MAX_KP) / (float) 100;
  cd.kI = mapFloat(cd.kI, MAX_ANALOG_VALUE, MIN_ANALOG_VALUE, MIN_KI, MAX_KI) / (float) 100;
  cd.kD = mapFloat(cd.kD, MAX_ANALOG_VALUE, MIN_ANALOG_VALUE, MIN_KD, MAX_KD) / (float) 100;

  #ifdef DEBUG_PID_VALUES
    Serial.print(F("kP: "));
    Serial.print(cd.kP, 4);
    Serial.print(F("\tkI: "));
    Serial.print(cd.kI, 4);
    Serial.print(F("\tkD: "));
    Serial.print(cd.kD, 4);
  #endif
}

void Controller::sendCalibrationData() {
  calibrationData[0] = cd.kP;
  calibrationData[1] = cd.kI;
  calibrationData[2] = cd.kD;
  calibrationData[3] = cd.reset;
  if (radio->write(&calibrationData, sizeCalibrationData)) {
    #ifdef DEBUG_RADIO
      Serial.println(F("Data sent"));
    #endif
  }
  else {
    #ifdef DEBUG_RADIO
      Serial.println(F("Failed transmitting calibration"));
    #endif
  }
}

void Controller::readResetButton() {
  int currentButtonReset = digitalRead(BUTTON_RESET);
  if (currentButtonReset != lastButtonReset) {
    lastButtonReset = currentButtonReset;
    buttonReset++;
    if (buttonReset % 4 == 0) {
      cd.reset = !cd.reset;
    }
  }
  #ifdef DEBUG_PID_VALUES
    Serial.print(F("\tReset: "));
    Serial.println(cd.reset);
  #endif
}

// CONTROL
void Controller::readJoystick1() {
  sp.throttle = MAX_ANALOG_VALUE - analogRead(j1.pinY);
  sp.yaw = analogRead(j1.pinX);
}

void Controller::readJoystick2() {
  sp.roll = analogRead(j2.pinX);
  sp.pitch = MAX_ANALOG_VALUE - analogRead(j2.pinY);
}

void Controller::readButtons() {
  int currentButtonStatus = digitalRead(BUTTON_STATUS);
  if (currentButtonStatus != lastButtonStatus) {
    lastButtonStatus = currentButtonStatus;
    buttonStatus++;
    if (buttonStatus % 2 == 0) {
      sp.status = !sp.status;
      // Quadcopter off
      if (!sp.status) {
        sp.holdDistance = false;
        sp.holdAltitude = false;
        return;
      }
    }
  }

  // Quadcopter on
  if (sp.status) {
    int currentButtonHoldDistance = digitalRead(BUTTON_HOLD_DISTANCE);
    if (currentButtonHoldDistance != lastButtonHoldDistance) {
      lastButtonHoldDistance = currentButtonHoldDistance;
      buttonHoldDistance++;
      if (buttonHoldDistance % 2 == 0) {
        sp.holdDistance = !sp.holdDistance;
      }
      if (sp.holdAltitude && sp.holdDistance) {
        sp.holdDistance = false;
      }
    }
    int currentButtonHoldAltitude = digitalRead(BUTTON_HOLD_ALTITUDE);
    if (currentButtonHoldAltitude != lastButtonHoldAltitude) {
      lastButtonHoldAltitude = currentButtonHoldAltitude;
      buttonHoldAltitude++;
      if (buttonHoldAltitude % 2 == 0) {
        sp.holdAltitude = !sp.holdAltitude;
      }
      if (sp.holdDistance && sp.holdAltitude) {
        sp.holdAltitude = false;
      }
    }
  }
 
}

void Controller::updateLeds() {
  digitalWrite(LED_STATUS, sp.status);
  digitalWrite(LED_HOLD_DISTANCE, sp.holdDistance);
  digitalWrite(LED_HOLD_ALTITUDE, sp.holdAltitude);
}

void Controller::getControllerData() {
  readJoystick1();
  readJoystick2();
  readButtons();
  printSetpoints();
  updateLeds();
}
