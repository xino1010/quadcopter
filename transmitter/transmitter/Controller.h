#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include "RF24.h"

#define NORMAL_MODE
//#define CALIBRATION_MODE

#define DEBUG
#ifdef DEBUG
  #ifdef NORMAL_MODE
    #define DEBUG_JOYSTICKS
  #endif
  //#define DEBUG_RADIO
  #ifdef CALIBRATION_MODE
    //#define DEBUG_POTENTIOMETERS
    #define DEBUG_PID_VALUES
  #endif
#endif

// TRANSMITTER
#define MIN_ANALOG_VALUE 0
#define MEDIUM_ANALOG_VALUE 512
#define MAX_ANALOG_VALUE 1023

// RADIO
#define NFR24L01_CE 9
#define NFR24L01_CSN 10

// CALIBRATION
#define BUTTON_RESET 8
#define POTENTIOMETRE_KP A1
#define POTENTIOMETRE_KI A2
#define POTENTIOMETRE_KD A3

// CONTROL
#define LED_STATUS 2
#define LED_HOLD_DISTANCE 4
#define LED_HOLD_ALTITUDE 6
#define BUTTON_STATUS 3
#define BUTTON_HOLD_DISTANCE 5
#define BUTTON_HOLD_ALTITUDE 7

class Controller {

  private:

    float MIN_KP = 0;
    float MAX_KP = 4;
    float MIN_KI = 0;
    float MAX_KI = 1;
    float MIN_KD = 0;
    float MAX_KD = 100;

    float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);

    // TRANSMITTER
    struct SetPoints {
      int throttle;
      float pitch;
      float roll;
      float yaw;
      int status;
      int holdDistance;
      int holdAltitude;
    };

    SetPoints sp;

    void printSetpoints();

    // RADIO
    const byte address[5] = {'c', 'a', 'n', 'a', 'l'};
    RF24 *radio;
    int radioData[7];
    const int sizeRadioData = sizeof(radioData);

    // CALIBRATION
    struct CalibrationData {
      float kP;
      float kI;
      float kD;
      int reset;
    };

    CalibrationData cd;
    float calibrationData[4];
    const int sizeCalibrationData = sizeof(calibrationData);
    int lastButtonReset;
    int buttonReset;

    // CONTROL
    struct JoystickInfo {
      int pinX;
      int pinY;
    };

    JoystickInfo j1;
    JoystickInfo j2;
    int lastButtonStatus;
    int lastButtonHoldDistance;
    int lastButtonHoldAltitude;
    int buttonStatus;
    int buttonHoldDistance;
    int buttonHoldAltitude;

    void readJoystick1();
    void readJoystick2();
    void readButtons();
    void updateLeds();

  public:
    Controller();

    // TRANSMITTER

    // RADIO
    void sendRadioInfo();

    // CALIBRATION
    void readPotentiometers();
    void sendCalibrationData();
    void readResetButton();

    // CONTROL
    void calibrateJoysticks();
    void getControllerData();
};
