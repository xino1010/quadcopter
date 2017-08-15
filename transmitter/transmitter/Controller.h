#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include "RF24.h"

#define DEBUG
#ifdef DEBUG
  //#define DEBUG_JOYSTICKS
  //#define DEBUG_BUTTONS
  #define DEBUG_RADIO
#endif

#define NORMAL_MODE

//#define CALIBRATION_MODE

// TRANSMITTER
#define MIN_ANALOG_VALUE 0
#define MEDIUM_ANALOG_VALUE 511
#define MAX_ANALOG_VALUE 1023
#define ZERO_THROTTLE 1000
#define MIN_THROTTLE 1100
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
#define NFR24L01_CE 9
#define NFR24L01_CSN 10

// CALIBRATION
#define POTENTIOMETRE_KP A1
#define POTENTIOMETRE_KI A2
#define POTENTIOMETRE_KD A3
#define RESET_BUTTON 2
#define MIN_KP 0.0
#define MAX_KP 3.0
#define MIN_KI 0.0
#define MAX_KI 1.0
#define MIN_KD 0.0
#define MAX_KD 100.0

// CONTROL
#define LED_STATUS 2
#define LED_HOLD_DISTANCE 3
#define LED_HOLD_ALTITUDE 4
#define BUTTON_STATUS 6
#define BUTTON_HOLD_DISTANCE 7
#define BUTTON_HOLD_ALTITUDE 8
#define NUMBER_READS_GET_OFFSET_JOYSTICK 400

class Controller {

  private:

    float mapFloat(long x, long in_min, long in_max, long out_min, long out_max);

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

    // CONTROL
    struct JoystickInfo {
      int num;
      int pinX;
      int pinY;
      int valX;
      int valY;
      int offsetX;
      int offsetY;
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
    void printJoystickData(struct JoystickInfo *ji);
    void readButtons();
    void printButtons();
    void updateLeds();

  public:
    Controller();

    // TRANSMITTER
    void calculateSetpoints();

    // RADIO
    void sendRadioInfo();

    // CALIBRATION
    void readPotentiometers();
    void sendCalibrationData();

    // CONTROL
    void calibrateJoysticks();
    void getControllerData();
};
