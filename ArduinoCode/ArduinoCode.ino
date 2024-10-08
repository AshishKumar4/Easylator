// Code Written Specifically for Atmega 328 variants i.e, Arduino Nano/Uno etc.
// Other Architecture support coming soon...
// Author: Ashish Kumar Singh, ashishkmr472@gmail.com

#include <Arduino.h>
#include <Wire.h>


// #define BMP180
// #define SENSOR_PRESSURE_TYPE BMP180

// #define STEPPER_ACCELSTEPPER
// #define STEPPER   STEPPER_ACCELSTEPPER


/*************************************************************************************************************************/
/*************************************** Registers to hold Important Values **********************************************/
/*************************************************************************************************************************/

////////////////////////////
// General Mode Registers-->
////////////////////////////

enum OperationMode
{
  OPERATION_NORMAL,
  OPERATION_PROGRAM,
  OPERATION_CALIBRATION
};

OperationMode OPERATION_MODE = OPERATION_NORMAL;
void (*OPERATION_FUNC)();

////////////////////////////////
// Ventilation Mode Registers-->
////////////////////////////////

enum VentilationMode
{
  VENTILATION_CONTROLLED, // i.e, PC
  VENTILATION_ASSISTED,   // i.e, AC
};

VentilationMode VENTILATION_MODE = VENTILATION_ASSISTED;
void (*VENTILATION_FUNC)() = 0;

// int CONF_VENTILATION_ASSISTED_BREATEWAIT_TIMEOUT = 0;

/////////////////////////////
// Potentiometer Registers-->
/////////////////////////////

enum Controls
{
  CONTROLS_BPM,
  CONTROLS_VOL,
  CONTROLS_PRES,
  CONTROLS_IE,
  CONTROLS_size
};

float REG_POT[int(CONTROLS_size)] = {0, 0, 0, 0};

///////////////////////
// General Registers-->
///////////////////////

// float REG_SENSOR_FLOW = 0;
// float REG_SENSOR_PRESSURE = 0;

/*************************************************************************************************************************/
/******************************************** Registers to hold Pin Values ***********************************************/
/*************************************************************************************************************************/

uint8_t PIN_ALARM = 10;

uint8_t PIN_POT[int(CONTROLS_size)] = {A0, A1, A2, A3};

uint8_t PIN_SWITCH_MODE_OPERATION = 4;
uint8_t PIN_SWITCH_MODE_VENTILATION = 7;

uint8_t PIN_SENSOR_PRESSURE;
uint8_t PIN_SENSOR_FLOW;

uint8_t PIN_STEPPER_STEP = 2;
uint8_t PIN_STEPPER_DIR = 3;

enum PinModes
{
  PINMODES_ANALOG = 0 << 0,
  PINMODES_DIGITAL = 1 << 0,
  PINMODES_PWM = 1 << 1,
  PINMODES_INPUT = 0 << 2,
  PINMODES_OUTPUT = 1 << 2
};

#define PINMODE_IO (1 << 2)
#define PINMODE_METHOD 3
uint8_t MAP_PIN_MODE[256];

/*************************************************************************************************************************/
/********************************************** General Configurations ***************************************************/
/*************************************************************************************************************************/

#define DEFAULT_BAUDRATE 57600

#define FLAG_CONF_FIRSTRUN  1 << 1

typedef struct Configurations
{ 
  // First byte is always FLAGS!
  uint8_t   FLAGS;

  uint16_t CONF_LOOP_PERIOD = 0;
  uint16_t CONF_VENTILATION_ASSISTED_BREATEWAIT_TIMEOUT = 0;

  uint16_t CONF_VENTILATION_ASSISTED_PRESSURE_TRIGGER_THRESHOLD = 1;

  // DEFINE THESE SHITS BELOW !!!
  float TIME_INHALE_HOLDUP = 1;
  float TIME_EXHALE_HOLDUP = 1;
  float CONF_DISTANCE_MAX = 10;                   //  DEFINE THIS SHIT
  float CONF_DISTANCE_MIN = 20;                   // DEFINE THIS SHIT
  float CONF_STEPPER_STEP_PER_REVOLUTION = 200;   // DEFINE THIS SHIT
  float CONF_STEPPER_DISTANCE_PER_REVOLUTION = 5; // DEFINE THIS SHIT

  uint16_t CONF_POT_MAX[int(CONTROLS_size)] = {1024, 1024, 1024, 1024};
  uint16_t CONF_POT_MIN[int(CONTROLS_size)] = {1, 1, 1, 1};
  uint16_t CONF_POT_MID[int(CONTROLS_size)] = {512, 512, 512, 512};

  float CONF_interm_LFACTORS[int(CONTROLS_size)];
  float CONF_interm_RFACTORS[int(CONTROLS_size)];

} Configurations_t;

Configurations_t GlobalConfigs;

/*************************************************************************************************************************/
/******************************************** Hardware Abstraction Layer *************************************************/
/*************************************************************************************************************************/

///////////////////////////////////
// Printing/UserInterface Logic -->
///////////////////////////////////

#define print(str)   (Serial.println(F(str)))

// void printVal str)
// {
//   Serial.println(str);
// }

void printVal(float val)
{
  Serial.println(String(val));
}

void printVal(int val)
{
  Serial.println(String((float)val));
}

void printVal(uint8_t val)
{
  Serial.println(String((float)val));
}

void printVal(uint16_t val)
{
  Serial.println(String((float)val));
}

void printerSetup()
{
  Serial.begin(DEFAULT_BAUDRATE);
}

String readString()
{
  return Serial.readString();
}

int readInt()
{
  while(!Serial.available());
  return Serial.parseInt();
}

int readFloat()
{
  while(!Serial.available());
  return Serial.parseFloat();
}

////////////////////////////////////////////
// Sensors Interfacing/Abstraction Logic -->
////////////////////////////////////////////

// For BMP180 -->

#if defined(BMP180)
// Uses the https://bitbucket.org/christandlg/bmp180mi/src/master/ BMP180MI library by grigor
#include <BMP180I2C.h>
#define BMP_I2C_ADDRESS 0x77

BMP180I2C bmp180(BMP_I2C_ADDRESS);

void sensorPressureInit()
{
  //begin() initializes the interface, checks the sensor ID and reads the calibration parameters.
  if (!bmp180.begin())
  {
    print("begin() failed. check your BMP180 Interface and I2C Address.");
    while (1)
      ;
  }

  //reset sensor to default parameters.
  bmp180.resetToDefaults();

  //enable ultra high resolution mode for pressure measurements
  bmp180.setSamplingMode(BMP180MI::MODE_UHR);
}

float OLD_PRESSURE = 0;

float fetchSensorsPressure()
{
  if (!bmp180.measurePressure())
  {
    print("could not start perssure measurement, is a measurement already running?");
    return OLD_PRESSURE;
  }

  //wait for the measurement to finish. proceed as soon as hasValue() returned true.
  do
  {
    delay(100);
  } while (!bmp180.hasValue());

  OLD_PRESSURE = bmp180.getPressure();
  return OLD_PRESSURE;
}

#else 

void sensorPressureInit()
{
  print("Pressure Sensor Not used!");
}


float fetchSensorsPressure()
{
  return 0;
}

#endif
//////////////////////////////////////////
// Motor Interfacing/Abstraction Logic -->
//////////////////////////////////////////

#if defined(STEPPER_ACCELSTEPPER)

#include "AccelStepper.h"

AccelStepper *stepperDriver;

void actuateStepperForwardBlock(float distance, float timePeriod)
{
  float rotationsNeeded = distance / GlobalConfigs.CONF_STEPPER_DISTANCE_PER_REVOLUTION;
  float stepsNeeded = rotationsNeeded * GlobalConfigs.CONF_STEPPER_STEP_PER_REVOLUTION;
  float stepSpeed = stepsNeeded / timePeriod;
  //  stepperDriver->distanceToGo(stepsNeeded);
  stepperDriver->move(stepsNeeded);
  stepperDriver->setMaxSpeed(stepSpeed);
  stepperDriver->setSpeed(stepSpeed);
  stepperDriver->runSpeedToPosition();
  delay(timePeriod);
  stepperDriver->stop();
}

void actuateStepperBackwardBlock(float distance, float timePeriod)
{
  float rotationsNeeded = distance / GlobalConfigs.CONF_STEPPER_DISTANCE_PER_REVOLUTION;
  float stepsNeeded = rotationsNeeded * GlobalConfigs.CONF_STEPPER_STEP_PER_REVOLUTION;
  float stepSpeed = stepsNeeded / timePeriod;
  //  stepperDriver->distanceToGo(-stepsNeeded);
  stepperDriver->move(-stepsNeeded - 1); // -1 for compensation
  stepperDriver->setMaxSpeed(-stepSpeed);
  stepperDriver->setSpeed(-stepSpeed);
  stepperDriver->runSpeedToPosition();
  delay(timePeriod);
  stepperDriver->stop();
}

void actuationInit()
{
  stepperDriver = new AccelStepper(1, PIN_STEPPER_STEP, PIN_STEPPER_DIR);
}

#else 

void actuateStepperForwardBlock(float distance, float timePeriod)
{
  print("Actuate Stepper forward for distance, timePeriod: ");
  printVal(distance);
  printVal(timePeriod);
}

void actuateStepperBackwardBlock(float distance, float timePeriod)
{
  print("Actuate Stepper Backward for distance, timePeriod: ");
  printVal(distance);
  printVal(timePeriod);
}

void actuationInit()
{
  print("Actuator not used!");
}

#endif

//////////////////
// Alarm Logic -->
//////////////////

void sendAlarm(int counts = 0, int toneDelay = 0, int freq = 1000, int toneWidth = 60)
{
  for (int i = 0; i < counts; i++)
  {
    // Start with 2 beeps
    tone(PIN_ALARM, freq);
    delay(toneWidth);
    tone(PIN_ALARM, 0);
    delay(toneDelay);
  }
  tone(PIN_ALARM, 0);
}

////////////////////////////
// Pin level Abstraction -->
////////////////////////////

int getPinValue(uint8_t pin, int mode = -1)
{
  int val = 0;
  if (mode == -1)
  {
    mode = (MAP_PIN_MODE[pin] & PINMODE_METHOD);
  }
  switch (mode)
  {
  case (int)PINMODES_ANALOG:
    val = analogRead(pin);
    break;
  case (int)PINMODES_DIGITAL:
    val = digitalRead(pin);
    break;
  case (int)PINMODES_PWM:
    val = pulseIn(pin, HIGH);
    break;
  default:
    // print("ERROR: WRONG PINMODE ENTERED!!!!");
    // return -1;
    val = -1;
  }
  return val;
}

void registerPin(uint8_t pin, uint8_t mode)
{
  MAP_PIN_MODE[pin] = mode;
  if (mode & PINMODE_IO == (int)PINMODES_INPUT)
  {
    pinMode(pin, INPUT);
  }
  else
  {
    pinMode(pin, OUTPUT);
  }
}

//////////////////////////////////////////////////
// Configuration Load/Save to Persistent Media -->
//////////////////////////////////////////////////

#include <EEPROM.h>

void loadConfigFromEEPROM(void *config)
{
  print("\n\tLoading config from EEPROM");

  uint8_t *confPtr = (uint8_t *)config;
  for (int i = 0; i < sizeof(Configurations_t); i++)
  {
    confPtr[i] = EEPROM.read(i);
  }

  print("\n\tLoading configs completed successfully!");
}

void saveConfigToEEPROM(void *config)
{
  print("\n\tSaving config to EEPROM");

  uint8_t *confPtr = (uint8_t *)config;
  for (int i = 0; i < sizeof(Configurations_t); i++)
  {
    EEPROM.write(i, confPtr[i]);
  }
  print("\n\tSaving Complete...Verifying...");

  for (int i = 0; i < sizeof(Configurations_t); i++)
  {
    if (EEPROM.read(i) != confPtr[i])
    {
      print("\n\tERROR! Configuration DOES NOT MATCH data saved to EEPROM!");
      panic();
    }
  }
}

/////////////////////////////////////////
// Hardware Initialization Procedures -->
/////////////////////////////////////////

void initHardware()
{
  printerSetup();
  print("\nHardware Initialization in progress...");
  // loadConfigFromEEPROM((void*)&GlobalConfigs);
  uint8_t ConfFlags = EEPROM.read(0);
  if(!(ConfFlags & FLAG_CONF_FIRSTRUN))
  {
    // This is the first time the system is online, revert to default configs
    saveConfigToEEPROM((void*)&GlobalConfigs);
    EEPROM.write(0, ConfFlags | FLAG_CONF_FIRSTRUN);
  }
  else 
  {
    loadConfigFromEEPROM((void*)&GlobalConfigs);
  }
  actuationInit();
  sensorPressureInit();
  registerPin(PIN_ALARM, (int)PINMODES_OUTPUT | (int)PINMODES_DIGITAL);
  for (int i = 0; i < (int)CONTROLS_size; i++)
  {
    registerPin(PIN_POT[i], (int)PINMODES_INPUT | (int)PINMODES_ANALOG);
  }
  registerPin(PIN_SWITCH_MODE_OPERATION, (int)PINMODES_INPUT | (int)PINMODES_DIGITAL);
  registerPin(PIN_SWITCH_MODE_VENTILATION, (int)PINMODES_INPUT | (int)PINMODES_DIGITAL);
  // registerPin(PIN_SWITCH_MODE_OPERATION, (int)PINMODES_INPUT | (int)PINMODES_DIGITAL);
}

/*************************************************************************************************************************/
/***************************************************** Main Code *********************************************************/
/*************************************************************************************************************************/

///////////////////////////////////////////
// Utility Functions, Logics, Equations -->
///////////////////////////////////////////

#define Stringify(Variable) (#Variable)

void panic()
{
  sendAlarm(10, 200, 3000);
}

inline float clamp(float val, float imin, float imax, float omin, float omax)
{
  float scaled = omin + (((omax - omin) / (imax - imin)) * (val - imin));
  return scaled;
}

void alarm_LowPlateauPressure()
{
  sendAlarm(5, 100, 2000);
}

void alarm_NoPatientResponse()
{
  sendAlarm(4, 50, 2000);
}

// The Length of time (in seconds) of an inhale/exhale cycle
inline float getTotalPeriod(float bmp)
{
  return 60.0 / (bmp);
}

// The Length of time (in seconds) of the inspiratory phase
inline float getInspiratoryPeriod(float totalPeriod, float ie)
{
  return totalPeriod / (1 + ie);
}

// The Length of time (in seconds) of the expiratory phase
inline float getExpiratoryPeriod(float totalPeriod, float inspiratoryPeriod)
{
  return totalPeriod - inspiratoryPeriod;
}

// The needed distance to cover for V Volume
inline float getRequiredDisplacement(float volumeNeeded)
{
  // Hypothesis -> The Relation is LINEAR!
  return clamp(volumeNeeded, 0, 100, GlobalConfigs.CONF_DISTANCE_MIN, GlobalConfigs.CONF_DISTANCE_MAX);
}

inline float getRequiredVelocity(float timePeriod, float distance)
{
  return distance / timePeriod;
}

int getControlValue(int controlId)
{
  float val = float(getPinValue(PIN_POT[controlId]));
  // printVal(val);
  // Eq --> ((no-ni)/(bo-bi))*(a-bi) + ni; (no-ni)*(bo-bi) is our factor
  // We take into account the mid stick values

  if (val <= (float)GlobalConfigs.CONF_POT_MID[controlId])
  {
    val = ((val - (float)GlobalConfigs.CONF_POT_MIN[controlId]) * GlobalConfigs.CONF_interm_LFACTORS[controlId]);
  }
  else
  {
    val = ((val - (float)GlobalConfigs.CONF_POT_MID[controlId]) * GlobalConfigs.CONF_interm_RFACTORS[controlId]) + 127.5;
  }
  if (val < 0)
    val = 0;
  else if (val > 255)
    val = 255;
  return int(val);
}

int getOperationalMode()
{
  // Get pin value of PIN_SWITCH_MODE_OPERATION and PIN_SWITCH_MODE_VENTILATION.
  int operationMode = getPinValue(PIN_SWITCH_MODE_OPERATION);
  int ventilationMode = getPinValue(PIN_SWITCH_MODE_VENTILATION);

  if (ventilationMode)
  {
    if (operationMode)
    {
      // This is Calibration Mode!
      return (int)OPERATION_CALIBRATION;
    }
    else
    {
      // This is Controlled Ventilation Mode i.e, PC Mode
      VENTILATION_FUNC = &Logic_ControlledVentilation;
      VENTILATION_MODE = VENTILATION_CONTROLLED;
      return (int)OPERATION_NORMAL;
    }
  }
  else
  {
    if (operationMode)
    {
      // This is Programming mode!
      return (int)OPERATION_PROGRAM;
    }
    else
    {
      // This is Assisted Ventilation Mode i.e, AC Mode
      VENTILATION_FUNC = &Logic_AssistedVentilation;
      VENTILATION_MODE == VENTILATION_ASSISTED;
      return (int)OPERATION_NORMAL;
    }
  }
}

/////////////////////////////////////////////////////
// Logic to fetch Dial settings, Switch modes etc -->
/////////////////////////////////////////////////////

inline void updateControls()
{
  // print("Raw Fetched Values---->");
  for (int i = 0; i < (int)CONTROLS_size; i++)
  {
    REG_POT[i] = getControlValue(i);
  }
}

inline float fetchControlsBPM()
{
  // Scale goes from 0 to 60 BPM, mapped to values 0 to 255
  return clamp(REG_POT[(int)CONTROLS_BPM], 0, 255, 0, 60);
}

inline float fetchControlsVOL()
{
  // Scale goes from 0 to 100 %, mapped to values 0 to 255
  return clamp(REG_POT[(int)CONTROLS_VOL], 0, 255, 0, 100);
}

inline float fetchControlsPRES()
{
  // Scale goes from 0 to 50cm H2O , mapped to values 0 to 255
  return clamp(REG_POT[(int)CONTROLS_BPM], 0, 255, 0, 50);
}

inline float fetchControlsIE()
{
  // Scale goes from 1 to 6 BPM, mapped to values 0 to 255
  return 1 / clamp(REG_POT[(int)CONTROLS_BPM], 0, 255, 1, 6);
}

/////////////////////////
// Calibration Logics -->
/////////////////////////

void CalibrateControls()
{
  // First operator should put all Pot dials to minimum values -- >
  // Send 2 alarm pulses seperated by 100ms to signify its ready for calibration
  print("\nReady for calibration...");
  print("\n\tSet all dials to minimum values in 10 seconds...");
  sendAlarm(2, 100);
  // Wait for 5 seconds
  delay(10000);
  // Get the POT values
  for (int i = 0; i < int(CONTROLS_size); i++)
  {
    GlobalConfigs.CONF_POT_MIN[i] = getPinValue(PIN_POT[i]);
  }

  print("\n\tSet all dials to maximum values in 10 seconds...");
  // Second, Operator needs to put all POTS to their maximum -->
  sendAlarm(1, 100);
  // Wait for 5 seconds
  delay(10000);
  // Get the POT values
  for (int i = 0; i < int(CONTROLS_size); i++)
  {
    GlobalConfigs.CONF_POT_MAX[i] = getPinValue(PIN_POT[i]);
  }

  print("\n\tSet all dials to middle values in 10 seconds...");
  sendAlarm(1, 100);
  // Wait for 5 seconds
  delay(10000);
  // Get the POT values
  for (int i = 0; i < int(CONTROLS_size); i++)
  {
    GlobalConfigs.CONF_POT_MID[i] = getPinValue(PIN_POT[i]);
  }

  print("\n\tGot Data, Calibrating...");
  sendAlarm(3, 200);

  // print("Want to reverse POT Direction?\n[1] Yes\n[2] No\n>");
  // int rev = readInt();
  // Incase you stupidly put wrong wiring ...
  for (int i = 0; i < 4; i++)
  {
    if (GlobalConfigs.CONF_POT_MIN[i] > GlobalConfigs.CONF_POT_MAX[i])
    {
      // Swap them
      int tmp = GlobalConfigs.CONF_POT_MIN[i];
      GlobalConfigs.CONF_POT_MIN[i] = GlobalConfigs.CONF_POT_MAX[i];
      GlobalConfigs.CONF_POT_MAX[i] = tmp;
    }
  }
  // Calibration Computation
  for (int i = 0; i < int(CONTROLS_size); i++)
  {
    if (GlobalConfigs.CONF_POT_MID[i] - GlobalConfigs.CONF_POT_MIN[i] != 0)
      GlobalConfigs.CONF_interm_LFACTORS[i] = (127.5 / float(GlobalConfigs.CONF_POT_MID[i] - GlobalConfigs.CONF_POT_MIN[i]));
    else
      GlobalConfigs.CONF_interm_LFACTORS[i] = (1);
    if (GlobalConfigs.CONF_POT_MAX[i] - GlobalConfigs.CONF_POT_MID[i] != 0)
      GlobalConfigs.CONF_interm_RFACTORS[i] = (127.5 / float(GlobalConfigs.CONF_POT_MAX[i] - GlobalConfigs.CONF_POT_MID[i]));
    else
      GlobalConfigs.CONF_interm_RFACTORS[i] = (1);
  }

    print("CONF_interm_LFACTORS is ");
    for (int i = 0; i < int(CONTROLS_size); i++)
    {
      printVal(GlobalConfigs.CONF_interm_LFACTORS[i]);
    }
    print("CONF_interm_RFACTORS is ");
    for (int i = 0; i < int(CONTROLS_size); i++)
    {
      printVal(GlobalConfigs.CONF_interm_RFACTORS[i]);
    }
  print("\n\tCalibration Completed...");
}

void CalibrationLogic()
{
  CalibrateControls();
  saveConfigToEEPROM((void*)&GlobalConfigs);
}

//////////////////////////////
// Serial Programmer Logic -->
//////////////////////////////

void ProgrammerSerial_editConfigLogic()
{
  print("Enter config variable to edit...");
  print("[1] CONF_LOOP_PERIOD");
  print("[2] CONF_VENTILATION_ASSISTED_BREATEWAIT_TIMEOUT");
  print("[3] CONF_VENTILATION_ASSISTED_PRESSURE_TRIGGER_THRESHOLD");
  print("[4] TIME_INHALE_HOLDUP");
  print("[5] TIME_EXHALE_HOLDUP");
  print("[6] CONF_DISTANCE_MAX");
  print("[7] CONF_DISTANCE_MIN");
  print("[8] CONF_STEPPER_STEP_PER_REVOLUTION");
  print("[9] CONF_STEPPER_DISTANCE_PER_REVOLUTION");
  print("[10] CONF_POT_MAX");
  print("[11] CONF_POT_MIN");
  print("[12] CONF_POT_MID");
  print("[13] CONF_interm_LFACTORS");
  print("[14] CONF_interm_RFACTORS");
  print("[15] return to main menu");
  print("Input> ");
  int opt = readInt();
  switch (opt)
  {
  case 1:
    print("CONF_LOOP_PERIOD is ");
    printVal(GlobalConfigs.CONF_LOOP_PERIOD);
    print("NewValue> ");
    GlobalConfigs.CONF_LOOP_PERIOD = (uint16_t)readInt();
    break;
  case 2:
    print("CONF_VENTILATION_ASSISTED_BREATEWAIT_TIMEOUT is ");
    printVal(GlobalConfigs.CONF_VENTILATION_ASSISTED_BREATEWAIT_TIMEOUT);
    print("NewValue> ");
    GlobalConfigs.CONF_VENTILATION_ASSISTED_BREATEWAIT_TIMEOUT = (uint16_t)readInt();
    break;
  case 3:
    print("CONF_VENTILATION_ASSISTED_PRESSURE_TRIGGER_THRESHOLD is ");
    printVal(GlobalConfigs.CONF_VENTILATION_ASSISTED_PRESSURE_TRIGGER_THRESHOLD);
    print("NewValue> ");
    GlobalConfigs.CONF_VENTILATION_ASSISTED_PRESSURE_TRIGGER_THRESHOLD = (uint16_t)readInt();
    break;
  case 4:
    print("TIME_INHALE_HOLDUP is ");
    printVal(GlobalConfigs.TIME_INHALE_HOLDUP);
    print("NewValue> ");
    GlobalConfigs.TIME_INHALE_HOLDUP = readFloat();
    break;
  case 5:
    print("TIME_EXHALE_HOLDUP is ");
    printVal(GlobalConfigs.TIME_EXHALE_HOLDUP);
    print("NewValue> ");
    GlobalConfigs.TIME_EXHALE_HOLDUP = readFloat();
    break;
  case 6:
    print("CONF_DISTANCE_MAX is ");
    printVal(GlobalConfigs.CONF_DISTANCE_MAX);
    print("NewValue> ");
    GlobalConfigs.CONF_DISTANCE_MAX = readFloat();
    break;
  case 7:
    print("CONF_DISTANCE_MIN is ");
    printVal(GlobalConfigs.CONF_DISTANCE_MIN);
    print("NewValue> ");
    GlobalConfigs.CONF_DISTANCE_MIN = readFloat();
    break;
  case 8:
    print("CONF_STEPPER_STEP_PER_REVOLUTION is ");
    printVal(GlobalConfigs.CONF_STEPPER_STEP_PER_REVOLUTION);
    print("NewValue> ");
    GlobalConfigs.CONF_STEPPER_STEP_PER_REVOLUTION = readFloat();
    break;
  case 9:
    print("CONF_STEPPER_DISTANCE_PER_REVOLUTION is ");
    printVal(GlobalConfigs.CONF_STEPPER_DISTANCE_PER_REVOLUTION);
    print("NewValue> ");
    GlobalConfigs.CONF_STEPPER_DISTANCE_PER_REVOLUTION = readFloat();
    break;
  case 10:
    print("CONF_POT_MAX is ");
    for (int i = 0; i < int(CONTROLS_size); i++)
    {
      printVal(GlobalConfigs.CONF_POT_MAX[i]);
    }
    print("NewValues> ");
    for (int i = 0; i < int(CONTROLS_size); i++)
    {
      GlobalConfigs.CONF_POT_MAX[i] = (uint16_t)readInt();
    }
    break;
  case 11:
    print("CONF_POT_MIN is ");
    for (int i = 0; i < int(CONTROLS_size); i++)
    {
      printVal(GlobalConfigs.CONF_POT_MIN[i]);
    }
    print("NewValues> ");
    for (int i = 0; i < int(CONTROLS_size); i++)
    {
      GlobalConfigs.CONF_POT_MIN[i] = (uint16_t)readInt();
    }
    break;
  case 12:
    print("CONF_POT_MID is ");
    for (int i = 0; i < int(CONTROLS_size); i++)
    {
      printVal(GlobalConfigs.CONF_POT_MID[i]);
    }
    print("NewValues> ");
    for (int i = 0; i < int(CONTROLS_size); i++)
    {
      GlobalConfigs.CONF_POT_MID[i] = (uint16_t)readInt();
    }
    break;
  case 13:
    print("CONF_interm_LFACTORS is ");
    for (int i = 0; i < int(CONTROLS_size); i++)
    {
      printVal(GlobalConfigs.CONF_interm_LFACTORS[i]);
    }
    print("NewValues> ");
    for (int i = 0; i < int(CONTROLS_size); i++)
    {
      GlobalConfigs.CONF_interm_LFACTORS[i] = readInt();
    }
    break;
  case 14:
    print("CONF_interm_RFACTORS is ");
    for (int i = 0; i < int(CONTROLS_size); i++)
    {
      printVal(GlobalConfigs.CONF_interm_RFACTORS[i]);
    }
    print("NewValues> ");
    for (int i = 0; i < int(CONTROLS_size); i++)
    {
      GlobalConfigs.CONF_interm_RFACTORS[i] = readInt();
    }
    break;
  case 15:
    return;
    break;
  default:
    print("Wrong Choice entered!");
  }
}

void ProgrammerSerial_displayConfigs()
{
  print("The Values are -->");
  print("CONF_LOOP_PERIOD is ");
  printVal(GlobalConfigs.CONF_LOOP_PERIOD);
  print("CONF_VENTILATION_ASSISTED_BREATEWAIT_TIMEOUT is ");
  printVal(GlobalConfigs.CONF_VENTILATION_ASSISTED_BREATEWAIT_TIMEOUT);
  print("CONF_VENTILATION_ASSISTED_PRESSURE_TRIGGER_THRESHOLD is ");
  printVal(GlobalConfigs.CONF_VENTILATION_ASSISTED_PRESSURE_TRIGGER_THRESHOLD);
  print("TIME_INHALE_HOLDUP is ");
  printVal(GlobalConfigs.TIME_INHALE_HOLDUP);
  print("TIME_EXHALE_HOLDUP is ");
  printVal(GlobalConfigs.TIME_EXHALE_HOLDUP);
  print("CONF_DISTANCE_MAX is ");
  printVal(GlobalConfigs.CONF_DISTANCE_MAX);
  print("CONF_DISTANCE_MIN is ");
  printVal(GlobalConfigs.CONF_DISTANCE_MIN);
  print("CONF_STEPPER_STEP_PER_REVOLUTION is ");
  printVal(GlobalConfigs.CONF_STEPPER_STEP_PER_REVOLUTION);
  print("CONF_STEPPER_DISTANCE_PER_REVOLUTION is ");
  printVal(GlobalConfigs.CONF_STEPPER_DISTANCE_PER_REVOLUTION);
  print("CONF_POT_MAX is ");
  for (int i = 0; i < int(CONTROLS_size); i++)
  {
    printVal(GlobalConfigs.CONF_POT_MAX[i]);
  }
  print("CONF_POT_MIN is ");
  for (int i = 0; i < int(CONTROLS_size); i++)
  {
    printVal(GlobalConfigs.CONF_POT_MIN[i]);
  }
  print("CONF_POT_MID is ");
  for (int i = 0; i < int(CONTROLS_size); i++)
  {
    printVal(GlobalConfigs.CONF_POT_MID[i]);
  }
  print("CONF_interm_LFACTORS is ");
  for (int i = 0; i < int(CONTROLS_size); i++)
  {
    printVal(GlobalConfigs.CONF_interm_LFACTORS[i]);
  }
  print("CONF_interm_RFACTORS is ");
  for (int i = 0; i < int(CONTROLS_size); i++)
  {
    printVal(GlobalConfigs.CONF_interm_RFACTORS[i]);
  }
}

void ProgrammerSerial_Logic()
{
  int state = 0;
  while (1)
  {
    print("\nWelcome to Main Menu\nEnter your choice...");
    print("[1] edit configuration");
    print("[2] save configuration");
    print("[3] load configuration");
    print("[4] show configuration");
    print("[5] reset configuration");
    print("[6] exit");
    print("Input> ");
    int opt = readInt();
    switch (opt)
    {
    case 1:
      ProgrammerSerial_editConfigLogic();
      break;
    case 2:
      saveConfigToEEPROM((void*)&GlobalConfigs);
      break;
    case 3:
      loadConfigFromEEPROM((void*)&GlobalConfigs);
      break;
    case 4:
      ProgrammerSerial_displayConfigs();
      break;
    case 5:
      {
        Configurations_t newConfigs;
        saveConfigToEEPROM((void*)&newConfigs);
      }
      break;
    case 6:
      print("Exiting...");
      return;
      break;
    default:
      print("Wrong Choice entered!");
    }
  }
}

//////////////////////////////////
// Ventilation Logic functions -->
//////////////////////////////////

void Ventilation_Assisted_Compression(float timePeriod, float linearDisplacement)
{
  // float requiredVelocity = getRequiredVelocity(timePeriod, linearDisplacement);
  // For now, Simple logic to simply move the motor to desired position
  actuateStepperForwardBlock(linearDisplacement, timePeriod); // This is a blocking call
}

void Ventilation_Assisted_InhaleHoldup(float timePeriod, float plateauPressureThreshold)
{
  float plateauPressure = fetchSensorsPressure();
  if (plateauPressure <= plateauPressureThreshold)
  {
    alarm_LowPlateauPressure();
  }
}

void Ventilation_Assisted_ExhaledHoldup(float timePeriod)
{
  // Right now, Do nothing...
}

void Ventilation_Assisted_Decompression(float timePeriod, float linearDisplacement)
{
  // float requiredVelocity = getRequiredVelocity(timePeriod, linearDisplacement);
  actuateStepperBackwardBlock(linearDisplacement, timePeriod);
}

int Ventilation_Assisted_PatientResponseLogic()
{
  // For RESPONSE_WAIT_TIME seconds, keep fetching Pressure, and
  // return if Pressure drops below GlobalConfigs.CONF_AC_MIN_PRESSURE
  float initialPressure = fetchSensorsPressure();
  
  for (int i = 0; i < 700; i++)
  {
    if (initialPressure - fetchSensorsPressure() <= GlobalConfigs.CONF_VENTILATION_ASSISTED_PRESSURE_TRIGGER_THRESHOLD)
    {
      return 0;
    }
    delay((int)(GlobalConfigs.CONF_VENTILATION_ASSISTED_BREATEWAIT_TIMEOUT)); // CUZ ((time(seconds) / 1000) * 1000) * 1000 = total loop time in miliseconds
  }
  return 1;
}

void Logic_AssistedVentilation()
{
  // Wait for Pressure Drop (Patient Trying to breath in), with a timeout of CONF_VENTILATION_ASSISTED_BREATEWAIT_TIMEOUT miliseconds
  int response = Ventilation_Assisted_PatientResponseLogic();
  if (response == 1)
  {
    // Fire up the alarms!
    alarm_NoPatientResponse();
  }

  float CurrentBPM = fetchControlsBPM();
  float ie = fetchControlsIE();
  float maxVolume = fetchControlsVOL();
  float plateauPressure = fetchControlsPRES();

  print("Fetched Values for BPM, IE, maxVolume, Pressure: ");
  printVal(CurrentBPM);
  printVal(ie);
  printVal(maxVolume);
  printVal(plateauPressure);

  float totalPeriod = getTotalPeriod(CurrentBPM);
  float inspiratoryPeriod = getInspiratoryPeriod(totalPeriod, ie);
  float expiratoryPeriod = getExpiratoryPeriod(totalPeriod, inspiratoryPeriod);
  float requiredLinearDistance = getRequiredDisplacement(maxVolume);

  // Compression Logic
  Ventilation_Assisted_Compression(inspiratoryPeriod, requiredLinearDistance);

  // Hold Inhaled Position
  Ventilation_Assisted_InhaleHoldup(GlobalConfigs.TIME_INHALE_HOLDUP, plateauPressure);

  // Decompression Logic
  Ventilation_Assisted_Decompression(expiratoryPeriod, requiredLinearDistance);

  // Hold Exhaled Logic
  Ventilation_Assisted_ExhaledHoldup(GlobalConfigs.TIME_EXHALE_HOLDUP);
}

void Logic_ControlledVentilation()
{
  // TODO: IMPLEMENT
}

////////////////////////////////////////////////////
// Main Execution Logics for all Operation modes -->
////////////////////////////////////////////////////

void Execution_Normal()
{
  updateControls();
  if (VENTILATION_FUNC != 0)
    VENTILATION_FUNC();
  else
    print("ERROR! Ventilation Function Pointer is NULL!");
}

void Execution_Program()
{
  //TODO: IMPLEMENT
  // This is a fast patch...
  print("Saving Configs to EEPROM...");
  // Configurations_t newConfigs;
  // saveConfigToEEPROM(&newConfigs)
  ProgrammerSerial_Logic();
}

void Execution_Calibration()
{
  CalibrationLogic();
  print("\nCalibration Complete! Turn off calibration mode, waiting for 10 seconds...");
  delay(10000);
  // while (1)
  //   ;
}

void ExecuteProperLogic()
{
  int mode = getOperationalMode();
  switch (mode)
  {
  case (int)OPERATION_NORMAL:
    print("\nExecuting in Normal Mode...");
    // OPERATION_FUNC = &Execution_Normal;
    Execution_Normal();
    break;
  case (int)OPERATION_PROGRAM:
    print("\nExecuting in Programing Mode...");
    // OPERATION_FUNC = &Execution_Program;
    Execution_Program();
    break;
  case (int)OPERATION_CALIBRATION:
    print("\nExecuting in Calibration Mode...");
    // OPERATION_FUNC = &Execution_Calibration;
    Execution_Calibration();
    break;
  default:
    print("ERROR! Wrong Operational Mode!");
    panic();
  }
}

///////////////////////
// Setup/Loop Logic -->
///////////////////////

void setup()
{
  // put your setup code here, to run once:
  initHardware();
}

void loop()
{
  // put your main code here, to run repeatedly:
  ExecuteProperLogic();
  if (GlobalConfigs.CONF_LOOP_PERIOD != 0)
    delay(GlobalConfigs.CONF_LOOP_PERIOD);
}
