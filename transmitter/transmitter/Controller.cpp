#include "Controller.h"

Controller::Controller() {
  // TRANSMITTER
  sp.throttle = MIN_THROTTLE;
  sp.pitch = MEDIUM_PITCH;
  sp.roll = MEDIUM_ROLL;
  sp.yaw = MEDIUM_YAW;
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

  j1.num = 1;
  j1.pinX = A0;
  j1.pinY = A1;
  j1.valX = MEDIUM_ANALOG_VALUE;
  j1.valY = 0;
  j1.offsetX = 0;
  j1.offsetY = 0;

  j2.num = 2;
  j2.pinX = A2;
  j2.pinY = A3;
  j2.valX = MEDIUM_ANALOG_VALUE;
  j2.valY = MEDIUM_ANALOG_VALUE;
  j2.offsetX = 0;
  j2.offsetY = 0;

  lastButtonStatus = LOW;
  lastButtonHoldDistance = LOW;
  lastButtonHoldAltitude = LOW;
  buttonStatus = 0;
  buttonHoldDistance = 0;
  buttonHoldAltitude = 0;

  #ifdef DEBUG
    Serial.println("Controller initialized");
  #endif

}

float Controller::mapFloat(long x, long in_min, long in_max, long out_min, long out_max) {
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

// TRANSMITTER
void Controller::calculateSetpoints() {
  // THROTTLE
  sp.throttle = map(j1.valY, MEDIUM_ANALOG_VALUE, MAX_ANALOG_VALUE, MIN_THROTTLE, MAX_THROTTLE);
  // PITCH
  sp.pitch = mapFloat(j2.valY, MIN_ANALOG_VALUE, MAX_ANALOG_VALUE, MAX_PITCH, MIN_PITCH);
  if (sp.pitch >= MEDIUM_PITCH - THRESHOLD && sp.pitch <= MEDIUM_PITCH + THRESHOLD) {
    sp.pitch = MEDIUM_PITCH;
  }
  // ROLL
  sp.roll = mapFloat(j2.valX, MIN_ANALOG_VALUE, MAX_ANALOG_VALUE, MIN_ROLL, MAX_ROLL);
  if (sp.roll >= MEDIUM_ROLL - THRESHOLD && sp.roll <= MEDIUM_ROLL + THRESHOLD) {
    sp.roll = MEDIUM_ROLL;
  }
  // YAW
  sp.yaw = mapFloat(j1.valX, MAX_ANALOG_VALUE, MIN_ANALOG_VALUE, MIN_YAW, MAX_YAW);
  if (sp.yaw >= MEDIUM_YAW - THRESHOLD && sp.yaw <= MEDIUM_YAW + THRESHOLD) {
    sp.yaw = MEDIUM_YAW;
  }

  printSetpoints();
}

void Controller::printSetpoints() {
  #ifdef DEBUG_JOYSTICKS
    Serial.print("Throttle: ");
    Serial.print(sp.throttle);
    Serial.print(" \tPitch: ");
    Serial.print(sp.pitch);
    Serial.print("º \tRoll: ");
    Serial.print(sp.roll);
    Serial.print("º \tYaw: ");
    Serial.print(sp.yaw);
    Serial.print("º \tStatus: ");
    Serial.print(sp.status);
    Serial.print(" \tHoldDist: ");
    Serial.print(sp.holdDistance);
    Serial.print(" \tHoldAlt: ");
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
      Serial.println("Data sent");
    #endif
  }
  else {
    #ifdef DEBUG_RADIO
      Serial.println("Failed transmitting setpoints");
    #endif
  }
}

// CALIBRATION
void Controller::readPotentiometers() {
  cd.kP = analogRead(POTENTIOMETRE_KP);
  cd.kI = analogRead(POTENTIOMETRE_KI);
  cd.kD = analogRead(POTENTIOMETRE_KD);
  #ifdef DEBUG_POTENTIOMETERS
    Serial.print("kP: ");
    Serial.print(cd.kP);
    Serial.print("\tkI: ");
    Serial.print(cd.kI);
    Serial.print("\tkD: ");
    Serial.println(cd.kD);
  #endif

  cd.kP = mapFloat(cd.kP, MAX_ANALOG_VALUE, MIN_ANALOG_VALUE, MIN_KP, MAX_KP);
  cd.kI = mapFloat(cd.kI, MAX_ANALOG_VALUE, MIN_ANALOG_VALUE, MIN_KI, MAX_KI);
  cd.kD = mapFloat(cd.kD, MAX_ANALOG_VALUE, MIN_ANALOG_VALUE, MIN_KD, MAX_KD);

  #ifdef DEBUG_PID_VALUES
    Serial.print("kP: ");
    Serial.print(cd.kP);
    Serial.print("\tkI: ");
    Serial.print(cd.kI);
    Serial.print("\tkD: ");
    Serial.print(cd.kD);
  #endif
}

void Controller::sendCalibrationData() {
  calibrationData[0] = cd.kP;
  calibrationData[1] = cd.kI;
  calibrationData[2] = cd.kD;
  calibrationData[3] = cd.reset;
  if (radio->write(&calibrationData, sizeCalibrationData)) {
    #ifdef DEBUG_RADIO
      Serial.println("Data sent");
    #endif
  }
  else {
    #ifdef DEBUG_RADIO
      Serial.println("Failed transmitting calibration");
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
    Serial.print("\tReset: ");
    Serial.println(cd.reset);
  #endif
}

// CONTROL
void Controller::readJoystick1() {
  j1.valX = analogRead(j1.pinX) + j1.offsetX;
  delay(10);
  j1.valY = constrain(analogRead(j1.pinY) + j1.offsetY, MEDIUM_ANALOG_VALUE, MAX_ANALOG_VALUE);
  delay(10);
}

void Controller::readJoystick2() {
  j2.valX = analogRead(j2.pinX) + j2.offsetX;
  delay(10);
  j2.valY = analogRead(j2.pinY) + j2.offsetY;
  delay(10);
}

void Controller::printJoystickData(struct JoystickInfo *ji) {
  /*
  #ifdef DEBUG_JOYSTICKS
    Serial.print("JOYSTICK ");
    Serial.print(ji->num);
    Serial.print(" -> X: ");
    Serial.print(ji->valX);
    Serial.print(" Y: ");
    Serial.println(ji->valY);
  #endif
  */
}

void Controller::readButtons() {
  int currentButtonStatus = digitalRead(BUTTON_STATUS);
  if (currentButtonStatus != lastButtonStatus) {
    lastButtonStatus = currentButtonStatus;
    buttonStatus++;
    if (buttonStatus % 2 == 0) {
      sp.status = !sp.status;
    }
  }
  int currentButtonHoldDistance = digitalRead(BUTTON_HOLD_DISTANCE);
  if (currentButtonHoldDistance != lastButtonHoldDistance) {
    lastButtonHoldDistance = currentButtonHoldDistance;
    buttonHoldDistance++;
    if (buttonHoldDistance % 2 == 0) {
      sp.holdDistance = !sp.holdDistance;
    }
  }
  int currentButtonHoldAltitude = digitalRead(BUTTON_HOLD_ALTITUDE);
  if (currentButtonHoldAltitude != lastButtonHoldAltitude) {
    lastButtonHoldAltitude = currentButtonHoldAltitude;
    buttonHoldAltitude++;
    if (buttonHoldAltitude % 2 == 0) {
      sp.holdAltitude = !sp.holdAltitude;
    }
  }
}

void Controller::printButtons() {
  #ifdef DEBUG_BUTTONS
    Serial.print("Status: ");
    Serial.print(sp.status);
    Serial.print("\tHoldDist: ");
    Serial.print(sp.holdDistance);
    Serial.print("\tHoldAlt: ");
    Serial.println(sp.holdAltitude);
  #endif
}

void Controller::updateLeds() {
  digitalWrite(LED_STATUS, sp.status);
  digitalWrite(LED_HOLD_DISTANCE, sp.holdDistance);
  digitalWrite(LED_HOLD_ALTITUDE, sp.holdAltitude);
}

void Controller::calibrateJoysticks() {
  #ifdef DEBUG
    Serial.println("Calibrating joysticks...");
  #endif

  double offsets[4] = { 0, 0, 0, 0 };
  for (int i = 0; i < NUMBER_READS_GET_OFFSET_JOYSTICK; i++) {
    offsets[0] += analogRead(j1.pinX);
    offsets[1] += analogRead(j1.pinY);
    offsets[2] += analogRead(j2.pinX);
    offsets[3] += analogRead(j2.pinY);
    delay(10);
  }
  int offset1x = (int) (offsets[0] / NUMBER_READS_GET_OFFSET_JOYSTICK);
  int offset1y = (int) (offsets[1] / NUMBER_READS_GET_OFFSET_JOYSTICK);
  int offset2x = (int) (offsets[2] / NUMBER_READS_GET_OFFSET_JOYSTICK);
  int offset2y = (int) (offsets[3] / NUMBER_READS_GET_OFFSET_JOYSTICK);

  #ifdef DEBUG
    Serial.print("J1X_AVG: ");
    Serial.print(offset1x);
    Serial.print("\tJ1Y_AVG: ");
    Serial.print(offset1y);
    Serial.print("\tJ2X_AVG: ");
    Serial.print(offset2x);
    Serial.print("\tJ2T_AVG: ");
    Serial.println(offset2y);
  #endif

  j1.offsetX = MEDIUM_ANALOG_VALUE - offset1x;
  j1.offsetY = MEDIUM_ANALOG_VALUE - offset1y;
  j2.offsetX = MEDIUM_ANALOG_VALUE - offset2x;
  j2.offsetY = MEDIUM_ANALOG_VALUE - offset2y;

  #ifdef DEBUG
    Serial.print("J1X_OFFSET: ");
    Serial.print(j1.offsetX);
    Serial.print("\tJ1Y_OFFSET: ");
    Serial.print(j1.offsetY);
    Serial.print("\tJ2X_OFFSET: ");
    Serial.print(j2.offsetX);
    Serial.print("\tJ2Y_OFFSET: ");
    Serial.println(j2.offsetY);
  #endif
}

void Controller::getControllerData() {
  readJoystick1();
  readJoystick2();
  readButtons();
  printJoystickData(&j1);
  printJoystickData(&j2);
  printButtons();
  updateLeds();
}
