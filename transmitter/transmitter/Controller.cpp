#include "Controller.h"

Controller::Controller() {

  // RADIO
  radio = new RF24(NFR24L01_CE, NFR24L01_CSN);
  radio->begin();
  radio->setDataRate(RF24_250KBPS);
  radio->setPALevel(RF24_PA_MAX); // RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
  radio->openWritingPipe(address);
  radio->stopListening();

  // CALIBRATION
  #ifdef CALIBRATION_MODE
    cd.kP = MIN_KP;
    cd.kI = MIN_KI;
    cd.kD = MIN_KD;
    cd.reset =  LOW;
    lastButtonReset = LOW;
    buttonReset = 0;
    pinMode(LED_RESET, OUTPUT);
    pinMode(BUTTON_RESET, INPUT);
  #endif

  #ifdef NORMAL_MODE
    // TRANSMITTER
    sp.throttle = MEDIUM_ANALOG_VALUE;
    sp.pitch = MEDIUM_ANALOG_VALUE;
    sp.roll = MEDIUM_ANALOG_VALUE;
    sp.yaw = MEDIUM_ANALOG_VALUE;
    sp.status = LOW;
    sp.holdDistance = LOW;
    sp.holdAltitude = LOW;
    
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
  
    // LCD
    lcd = new LiquidCrystal_I2C(0x3F, 16, 2);  // Inicia el LCD en la dirección 0x3F, con 16 caracteres y 2 líneas
    lcd->begin();
    lcd->backlight();
    lastShowAngles = millis();
  #endif

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

  cd.kP = mapFloat((float) cd.kP, MAX_ANALOG_VALUE, MIN_ANALOG_VALUE, MIN_KP, MAX_KP);
  cd.kI = mapFloat((float) cd.kI, MAX_ANALOG_VALUE, MIN_ANALOG_VALUE, MIN_KI, MAX_KI);
  cd.kD = mapFloat((float) cd.kD, MAX_ANALOG_VALUE, MIN_ANALOG_VALUE, MIN_KD, MAX_KD);

  #ifdef DEBUG_PID_VALUES
    Serial.print(F("kP: "));
    Serial.print(cd.kP, 3);
    Serial.print(F("\tkI: "));
    Serial.print(cd.kI, 3);
    Serial.print(F("\tkD: "));
    Serial.print(cd.kD, 3);
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
    if (buttonReset % 2 == 0) {
      cd.reset = !cd.reset;
      digitalWrite(LED_RESET, cd.reset);
    }
  }
  #ifdef DEBUG_PID_VALUES
    Serial.print(F("\tReset: "));
    Serial.println(cd.reset);
  #endif
}

// CONTROL
void Controller::readJoystick1() {
  sp.throttle = MAX_ANALOG_VALUE - analogRead(j1.pinX);
  sp.yaw = analogRead(j1.pinY);
}

void Controller::readJoystick2() {
  sp.pitch = MAX_ANALOG_VALUE - analogRead(j2.pinX);
  sp.roll = MAX_ANALOG_VALUE - analogRead(j2.pinY);
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

// LCD
void Controller::showAngles() {
  if (millis() - lastShowAngles > REFRESH_LCD) {
    lastShowAngles = millis();
    lcd->clear();
    lcd->setCursor(0, 0);
    lcd->print("TH:");
    lcd->print(map(sp.throttle, THROTTLE_MIN, THROTTLE_MAX, MIN_VALUE_MOTOR, MAX_VALUE_MOTOR));
    lcd->print(" YW: ");
    lcd->print(map(sp.yaw, YAW_RMIN, YAW_RMAX, YAW_WMIN, YAW_WMAX));
    lcd->setCursor(0, 1);
    lcd->print("PC: ");
    lcd->print(map(sp.pitch, PITCH_RMIN, PITCH_RMAX, PITCH_WMIN, PITCH_WMAX));
    lcd->print(" RL: ");
    lcd->print(map(sp.roll, ROLL_RMIN, ROLL_RMAX, ROLL_WMIN, ROLL_WMAX));
  }
}

