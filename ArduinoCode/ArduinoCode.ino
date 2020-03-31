// Code Written Specifically for Atmega 328 variants i.e, Arduino Nano/Uno etc.
// Other Architecture support coming soon...
// Author: Ashish Kumar Singh, ashishkmr472@gmail.com

#include <Arduino.h>
#include <Wire.h>

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
  VENTILATION_ASSISTED, // i.e, AC
};

VentilationMode VENTILATION_MODE = VENTILATION_ASSISTED;
void (*VENTILATION_FUNC)() = nullptr;

int CONF_VENTILATION_ASSISTED_BREATEWAIT_TIMEOUT = 0;

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

float REG_SENSOR_FLOW = 0;
float REG_SENSOR_PRESSURE = 0;

/*************************************************************************************************************************/
/******************************************** Registers to hold Pin Values ***********************************************/
/*************************************************************************************************************************/

int PIN_ALARM = 2;

int PIN_POT[int(CONTROLS_size)] = {A0, A1, A2, A3};

int PIN_SWITCH_MODE_OPERATION = 4;
int PIN_SWITCH_MODE_VENTILATION = 7;

int PIN_SENSOR_PRESSURE;
int PIN_SENSOR_FLOW;

int PIN_STEPPER_STEP = 2;
int PIN_STEPPER_DIR = 3;

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

#define DEFAULT_BAUDRATE      57600

typedef struct Configurations 
{
  uint16_t CONF_LOOP_PERIOD = 0;
  uint16_t CONF_VENTILATION_ASSISTED_BREATEWAIT_TIMEOUT = 0;

  uint16_t CONF_VENTILATION_ASSISTED_PRESSURE_TRIGGER_THRESHOLD = 5;

  // DEFINE THESE SHITS BELOW !!!
  float     TIME_INHALE_HOLDUP = 1;
  float     TIME_EXHALE_HOLDUP = 1;
  float     CONF_DISTANCE_MAX = 10;   //  DEFINE THIS SHIT
  float     CONF_DISTANCE_MIN = 20;   // DEFINE THIS SHIT    
  float     CONF_STEPPER_STEP_PER_REVOLUTION = 200;   // DEFINE THIS SHIT
  float     CONF_STEPPER_DISTANCE_PER_REVOLUTION = 5;     // DEFINE THIS SHIT

  uint16_t CONF_POT_MAX[int(CONTROLS_size)] = {1024, 1024, 1024, 1024};
  uint16_t CONF_POT_MIN[int(CONTROLS_size)] = {0, 0, 0, 0};
  uint16_t CONF_POT_MID[int(CONTROLS_size)] = {512, 512, 512, 512};

  float    CONF_interm_LFACTORS[int(CONTROLS_size)];
  float    CONF_interm_RFACTORS[int(CONTROLS_size)];
}Configurations_t;

Configurations_t GlobalConfigs;

/*************************************************************************************************************************/
/******************************************** Hardware Abstraction Layer *************************************************/
/*************************************************************************************************************************/

////////////////////////////////////////////
// Sensors Interfacing/Abstraction Logic -->
////////////////////////////////////////////

#define BMP180
#define SENSOR_PRESSURE_TYPE BMP180

// For BMP180 -->

#if defined(BMP180) 

#include <BMP180I2C.h>
#define BMP_I2C_ADDRESS 0x77

BMP180I2C bmp180(BMP_I2C_ADDRESS);

void sensorPressureInit()
{
	//begin() initializes the interface, checks the sensor ID and reads the calibration parameters.  
	if (!bmp180.begin())
	{
		Serial.println("begin() failed. check your BMP180 Interface and I2C Address.");
		while (1);
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

#endif
//////////////////////////////////////////
// Motor Interfacing/Abstraction Logic -->
//////////////////////////////////////////

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
  // Wait until rotationsNeeded are completed
  // for(int i = 0; i < 1000000; i++)  // Just to implement a form of timeout
  // {
  //   if(stepperDriver->currentPosition() >= stepsNeeded)
  //   {
  //     stepperDriver->stop();
  //     break;
  //   }
  // }
  delay(timePeriod);
  stepperDriver->stop();
}

void actuateStepperBackwardBlock(float distance, float timePeriod)
{
  float rotationsNeeded = distance / GlobalConfigs.CONF_STEPPER_DISTANCE_PER_REVOLUTION;
  float stepsNeeded = rotationsNeeded * GlobalConfigs.CONF_STEPPER_STEP_PER_REVOLUTION;
  float stepSpeed = stepsNeeded / timePeriod;
//  stepperDriver->distanceToGo(-stepsNeeded);
  stepperDriver->move(-stepsNeeded - 1);  // -1 for compensation
  stepperDriver->setMaxSpeed(-stepSpeed);
  stepperDriver->setSpeed(-stepSpeed);
  stepperDriver->runSpeedToPosition();
  // Wait until rotationsNeeded are completed
  // for(int i = 0; i < 1000000; i++)  // Just to implement a form of timeout
  // {
  //   if(stepperDriver->currentPosition() >= stepsNeeded)
  //   {
  //     stepperDriver->stop();
  //     break;
  //   }
  // }
  delay(timePeriod);
  stepperDriver->stop();
}

void actuationInit()
{
  stepperDriver = new AccelStepper(1, PIN_STEPPER_STEP, PIN_STEPPER_DIR);
}

//////////////////
// Alarm Logic -->
//////////////////

void sendAlarm(int counts = 0, int toneDelay = 0, int freq = 1000)
{
  for (int i = 0; i < counts; i++)
  {
    // Start with 2 beeps
    tone(PIN_ALARM, freq);
    delay(toneDelay);
  }
}

///////////////////////////////////
// Printing/UserInterface Logic -->
///////////////////////////////////

void print(String str)
{
  Serial.println(str);
}

void printerSetup()
{
  Serial.begin(DEFAULT_BAUDRATE);
}

////////////////////////////
// Pin level Abstraction -->
////////////////////////////

int getPinValue(int pin, int mode = -1)
{
  int val = 0;
  if (mode == -1)
  {
    mode = MAP_PIN_MODE[pin] & PINMODE_METHOD;
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
    print("ERROR: WRONG PINMODE ENTERED!!!!");
    return -1;
  }
  return val;
}

void registerPin(int pin, uint8_t mode)
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

void loadConfigFromEEPROM()
{
  print("\n\tLoading config from EEPROM");

  uint8_t* confPtr = (uint8_t*)&GlobalConfigs;
  for(int i = 0; i < sizeof(Configurations_t); i++)
  {
    confPtr[i] = EEPROM.read(i);
  }

  print("\n\tLoading configs completed successfully!");
}

void saveConfigToEEPROM()
{
  print("\n\tSaving config to EEPROM");

  uint8_t* confPtr = (uint8_t*)&GlobalConfigs;
  for(int i = 0; i < sizeof(Configurations_t); i++)
  {
    EEPROM.write(i, confPtr[i]);
  }
  print("\n\tSaving Complete...Verifying...");

  for(int i = 0; i < sizeof(Configurations_t); i++)
  {
    if(EEPROM.read(i) != confPtr[i])
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
  loadConfigFromEEPROM();
  actuationInit();
  sensorPressureInit();
  registerPin(PIN_ALARM, (int)PINMODES_OUTPUT | (int)PINMODES_DIGITAL);
  for(int i = 0; i < (int)CONTROLS_size; i++)
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

int getControlValue(int pin)
{
  double val = double(getPinValue(pin));
  // Eq --> ((no-ni)/(bo-bi))*(a-bi) + ni; (no-ni)*(bo-bi) is our factor
  // We take into account the mid stick values
  if (val <= GlobalConfigs.CONF_POT_MID[pin])
  {
    val = ((val - GlobalConfigs.CONF_POT_MIN[pin]) * GlobalConfigs.CONF_interm_LFACTORS[pin]);
  }
  else
  {
    val = ((val - GlobalConfigs.CONF_POT_MID[pin]) * GlobalConfigs.CONF_interm_RFACTORS[pin]) + 127.5;
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

  if(ventilationMode)
  {
    if(operationMode)
    {
      // This is Calibration Mode!
      return (int)OPERATION_CALIBRATION;
    }
    else 
    {
      // This is Controlled Ventilation Mode i.e, PC Mode
      VENTILATION_MODE = VENTILATION_CONTROLLED;
      return (int)OPERATION_NORMAL;
    }
  }
  else 
  {
    if(operationMode)
    {
      // This is Programming mode! 
      return (int)OPERATION_PROGRAM;
    }
    else 
    {
      // This is Assisted Ventilation Mode i.e, AC Mode 
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
  for(int i = 0; i < (int)CONTROLS_size; i++)
  {
    REG_POT[i] = getControlValue(PIN_POT[i]);
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
  print("\n\tSet all dials to minimum values in 5 seconds...");
  sendAlarm(2, 100);
  // Wait for 5 seconds
  delay(5000);
  // Get the POT values
  for (int i = 0; i < int(CONTROLS_size); i++)
  {
    GlobalConfigs.CONF_POT_MIN[i] = getPinValue(PIN_POT[i]);
  }

  print("\n\tSet all dials to maximum values in 5 seconds...");
  // Second, Operator needs to put all POTS to their maximum -->
  sendAlarm(1, 100);
  // Wait for 5 seconds
  delay(5000);
  // Get the POT values
  for (int i = 0; i < int(CONTROLS_size); i++)
  {
    GlobalConfigs.CONF_POT_MAX[i] = getPinValue(PIN_POT[i]);
  }

  print("\n\tSet all dials to middle values in 5 seconds...");
  sendAlarm(1, 100);
  // Wait for 5 seconds
  delay(5000);
  // Get the POT values
  for (int i = 0; i < int(CONTROLS_size); i++)
  {
    GlobalConfigs.CONF_POT_MID[i] = getPinValue(PIN_POT[i]);
  }

  print("\n\tGot Data, Calibrating...");
  sendAlarm(3, 200);
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
      GlobalConfigs.CONF_interm_LFACTORS[i] = (127.5 / double(GlobalConfigs.CONF_POT_MID[i] - GlobalConfigs.CONF_POT_MIN[i]));
    else
      GlobalConfigs.CONF_interm_LFACTORS[i] = (1);
    if (GlobalConfigs.CONF_POT_MAX[i] - GlobalConfigs.CONF_POT_MID[i] != 0)
      GlobalConfigs.CONF_interm_RFACTORS[i] = (127.5 / double(GlobalConfigs.CONF_POT_MAX[i] - GlobalConfigs.CONF_POT_MID[i]));
    else
      GlobalConfigs.CONF_interm_RFACTORS[i] = (1);
  }

  print("\n\tCalibration Completed...");
}

void CalibrationLogic()
{
  CalibrateControls();
  saveConfigToEEPROM();
}

//////////////////////////////////
// Ventilation Logic functions -->
//////////////////////////////////

void Ventilation_Assisted_Compression(float timePeriod, float linearDisplacement)
{
  // float requiredVelocity = getRequiredVelocity(timePeriod, linearDisplacement);
  // For now, Simple logic to simply move the motor to desired position 
  actuateStepperForwardBlock(linearDisplacement, timePeriod);  // This is a blocking call
}

void Ventilation_Assisted_InhaleHoldup(float timePeriod, float plateauPressureThreshold)
{
  float plateauPressure = fetchSensorsPressure();
  if(plateauPressure <= plateauPressureThreshold)
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
  for(int i = 0; i < 700; i++)
  {
    if(initialPressure - fetchSensorsPressure() <= GlobalConfigs.CONF_VENTILATION_ASSISTED_PRESSURE_TRIGGER_THRESHOLD)
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
  if(VENTILATION_FUNC != nullptr)
    VENTILATION_FUNC();
  else 
    print("ERROR! Ventilation Function Pointer is NULL!");
}

void Execution_Program()
{
  //TODO: IMPLEMENT 
  // This is a fast patch...
  print("Saving Configs to EEPROM...");

  Configurations_t newConfigs;
  uint8_t* confPtr = (uint8_t*)&newConfigs;
  for(int i = 0; i < sizeof(Configurations_t); i++)
  {
    EEPROM.write(i, confPtr[i]);
  }
  print("\n\tSaving Complete...Verifying...");

  for(int i = 0; i < sizeof(Configurations_t); i++)
  {
    if(EEPROM.read(i) != confPtr[i])
    {
      print("\n\tERROR! Configuration DOES NOT MATCH data saved to EEPROM!");
      panic();
    }
  }
}

void Execution_Calibration()
{
  CalibrationLogic();
  print("\nCalibration Complete! You may now restart the device...");
  while(1);
}

void ExecuteProperLogic()
{
  int mode = getOperationalMode();
  switch(mode)
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
  if(GlobalConfigs.CONF_LOOP_PERIOD != 0)
    delay(GlobalConfigs.CONF_LOOP_PERIOD);
}
