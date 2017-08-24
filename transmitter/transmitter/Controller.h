#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include "RF24.h"
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

//#define NORMAL_MODE
#define CALIBRATION_MODE

#define DEBUG
#ifdef DEBUG
  #ifdef NORMAL_MODE
    //#define DEBUG_JOYSTICKS
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
#define LED_RESET A5
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

// LCD
#define REFRESH_LCD 250 // ms

// IMU
#define PITCH_RMIN 0
#define PITCH_RMEDIUM 512
#define PITCH_RMAX 1024
#define PITCH_WMIN -30
#define PITCH_WMAX 30
#define ROLL_RMIN 0
#define ROLL_RMEDIUM 512
#define ROLL_RMAX 1024
#define ROLL_WMIN -30
#define ROLL_WMAX 30
#define YAW_RMIN 0
#define YAW_RMEDIUM 512
#define YAW_RMAX 1024
#define YAW_WMIN 135
#define YAW_WMAX -135
#define ZERO_VALUE_MOTOR 1000
#define MIN_VALUE_MOTOR 1100
#define MAX_VALUE_MOTOR 1500
#define THROTTLE_MIN 512
#define THROTTLE_MAX 1023

class Controller {

  private:

    float MIN_KP = 0;
    float MAX_KP = 4;
    float MIN_KI = 0;
    float MAX_KI = 1;
    float MIN_KD = 0;
    float MAX_KD = 5;

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

    // LCD
    LiquidCrystal_I2C *lcd;
    unsigned long lastShowAngles;

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

    // LCD
    void showAngles();
};
