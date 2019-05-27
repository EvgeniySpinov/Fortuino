/*
  This code is solving this problem:
    - Installed gas equipment is showing gas level in separate indicator inside the car
    - Native gauge of gasoline becomes useless, since car is mostly used on gas and not benzine
    - It would be nice to show level of gas, instead of level of benzine, when car is working on gas and show benzine level, when car switching to benzine.
    - Bingo, this code is exactly to solve this problem

*/

#include <Arduino.h>
#include <SPI.h>
#include <stdarg.h>
#include <time.h>
// #include <mcp_can_dfs.h>
// #include <mcp_can.h>
#define INT32U unsigned long int
INT32U canId = 0x000;
#define VERSION "1.2.13"
#define DEBUG false

// Pin assignments
// Input - Analog
const int AUTO_LIGHT = A0;
const int AUTO_WIPER = A1;
const int GAS_LEVEL = A2;
const int GAS_VALVE = A3;

// Output - Digital
const int AUTO_WIPER_RELAY = 6;
const int FUEL_DASHBOARD_RELAY = 7;
const int AUTO_LIGHT_RELAY = 8;

// Output - Digital indication
const int AUTO_WIPER_RELAY_INDICATOR = 5;
const int AUTO_LIGHT_RELAY_INDICATOR = 3;
const int INDICATOR_VOLTAGE = 90; // 2.3V in %% from 5V


// set pin 9 as the slave select for the digital pot:
const int slaveSelectPin = 9;

// Step which we will put potentiometr to
int step = 0;

// Due to weird behavoir of measuing Dashboard in Fortuner, after switching on auto-wipers and auto-lights, 
// measurement of gas level is skewed only on dashboard. Other parameters are not affected. This variable is 
// a hotfix to this issue: we will try to increase resistance on skewed amount. Each item is adding 12.5% of skew.
// For example, without anything turned on, you see 25% of fuel left. 
// With autolights on: 37.5% left
// With both on: 50% left
int resistanceCorrection = 0;

// This is the step used to do resistance correction
const float RESISTANCE_CORRECTION_STEP = 0.5;

/** Gas related settings **/

bool isGasValveOpen = false;
const int AMOUNT_OF_MEASURES = 10;
const float VOLTAGE_DEVIATION = 0; // Value in voltage with difference to what LPG controller shows. It might be easier to debug using the same values
int gasValveCurrentMeasurement[AMOUNT_OF_MEASURES];
int gasValveCurrentMeasurementArrayPointer = 0;

int gasLevelCurrentMeasurement[AMOUNT_OF_MEASURES];
int gasLevelCurrentMeasurementArrayPointer = 0;

// Variables related to gas measurement

// Actual resistance is 120 Ohm and it works when fuel consumption is slow, however in case of moving on
// highway you can end up without a fuel when even light hasn't lighted up. Probably it's a question of 
// calibration, but I will try increasing resistance, so indication can start showing earlier.

// So the concept is the following: we use 120 Ohm, when fuel consumption is matching city pattern, and
// 130 Ohm, when it matches highway pattern. Defaults to 120 Ohm, as this is the main car mode.
int MaxFuelGaugeResistance = 120;
int gaugeResistanceSinceStart = 0;
const int MAX_FUEL_GAUGE_RESISTANCE_HIGHWAY = 135;

#define HIGHWAY_RESISTANCE_THRESHOLD 0.5 //%% of how much fuel should be consumed in 1 engine start before pattern of highway will be detected 
#define STEP_RESISTANCE 1.6
#define DIGIPOT_MINIMAL_RESISTANCE 10
#define MAX_SENSOR_VALUE 5
#define NUMBER_OF_CHANNELS_IN_DIGIPOT 6
#define NUMBER_OF_DEFECTIVE_CHANNELS_IN_DIGIPOT 1

/** State settings **/
int gasLevelResistanceStepSetPreviously = -1;
boolean autoWiperEnabled = false;
boolean autoLightEnabled = false; 
boolean blinkingIndicators = false; // Indicate if auto-wiper and auto-light should blink, cause were switched on together
boolean lightsAreOn = true; // Indicates if currently indicators are on

/** CAN BUS Shield related settings **/

#define CAN_500KBPS  16

// Set control ping for shield to pin #10
const int SPI_CS_PIN = 10;
// MCP_CAN CAN(SPI_CS_PIN);


/** Scheduler related settings **/

const int NUMBER_OF_SERVICES = 6;

// 5 is number of services by index:
#define GAS_VALVE_CHECKS 0
#define GAS_CAPACITY_MEASUREMENT 1
#define AUTO_WIPER_CHECKS 2
#define AUTO_LIGHT_CHECKS 3
#define PROJECTABLE_HUD_UPDATES 4
#define BLINKING_INDICATORS 5

// Last timestamps when services were ran
unsigned long lastRunTimes[NUMBER_OF_SERVICES] = {};

// List of tasks that are scheduled to run during current loop
int scheduledTasks[NUMBER_OF_SERVICES] = {};

// Cooldowns for tasks in milli seconds. High cooldown actually means that service is disabled.
// 0 - 3 seconds
// 1 - 100 milliseconds
// ...

// Timer to measure actual valve state right now
const int GAS_VALVE_CHECKS_HIGH_FREQUENCY_CHECKS = 100;

// Timer when valve status is determined
const int GAS_VALVE_CHECKS_LOW_FREQUENCY_CHECKS = 3000;

int taskCooldowns[NUMBER_OF_SERVICES] = {GAS_VALVE_CHECKS_LOW_FREQUENCY_CHECKS, 100, 1000, 1000, 999999999999, 300};
// int taskCooldowns[NUMBER_OF_SERVICES] = {99999999999999999, 99999999999999, 9999999999999, 999999999999, 1000};

// the setup routine runs once when you press reset:
void setup() {
  Serial.begin(115200);
  Serial.println();
  say("Entering initialization stage. Fortuino version: %s. Debug is %s", VERSION, (DEBUG ? "ON" : "OFF"));
  initGasMeter();
  initAutoWiper();
  initAutoLight();
  // initProjectableHUD();
}

void initGasMeter() {

  // initialize serial communication at 9600 bits per second:
  Serial.print(F("Initializing potentiometr AD5206 ... "));
  pinMode(slaveSelectPin, OUTPUT);
  pinMode(FUEL_DASHBOARD_RELAY, OUTPUT);
  SPI.begin();

  Serial.println(F(" complete."));
}

void initAutoWiper() {
  say("Initializing auto wiper ...");
  pinMode(AUTO_WIPER_RELAY, OUTPUT);
  pinMode(AUTO_WIPER_RELAY_INDICATOR, OUTPUT);
  digitalWrite(AUTO_WIPER_RELAY, LOW);
  digitalWrite(AUTO_WIPER_RELAY_INDICATOR, 0);
}
void initAutoLight() {
  say("Initializing auto light sensors ...");
  pinMode(AUTO_LIGHT_RELAY, OUTPUT);
  pinMode(AUTO_LIGHT_RELAY_INDICATOR, OUTPUT);
  digitalWrite(AUTO_LIGHT_RELAY, LOW);
  digitalWrite(AUTO_LIGHT_RELAY_INDICATOR, 0);
}

void initProjectableHUD() {
    // while (CAN_OK != CAN.begin(CAN_500KBPS))              // init can bus : baudrate = 500k
    // {
    //     Serial.println("CAN BUS Shield init fail");
    //     Serial.println(" Init CAN BUS Shield again");
    //     delay(100);
    // }
    // Serial.println("CAN BUS Shield init ok!");
}

// the loop routine runs over and over again forever:
void loop() {
  scheduleTasks();
  executeTasks();
}

// Function that schedules tasks to make sure that Arduino operates in almost real time
void scheduleTasks() {
  for(int i=0;i<NUMBER_OF_SERVICES;i++) {

    // Checking cooldowns
    if (lastRunTimes[i] + taskCooldowns[i] < millis()) {
      scheduledTasks[i] = 1;
    } else {
      scheduledTasks[i] = 0;
    } 
  }

  // if gas valve is closed - do not schedule updates to dashboard
  if (!isGasValveOpen) {
    scheduledTasks[GAS_CAPACITY_MEASUREMENT] = 0;
  }

  if (!blinkingIndicators) {
    scheduledTasks[BLINKING_INDICATORS] = 0;
  }
}

void executeTasks()
{
  for(int i=0;i<NUMBER_OF_SERVICES;i++) {
    if (scheduledTasks[i]) {
      switch (i) {
        case GAS_VALVE_CHECKS:
          lastRunTimes[GAS_VALVE_CHECKS] = millis();
          updateGasValveOpen();
          break;
        case GAS_CAPACITY_MEASUREMENT:
          lastRunTimes[GAS_CAPACITY_MEASUREMENT] = millis();
          doGasCapacityMeasurement();
          break;
        case AUTO_WIPER_CHECKS:
          lastRunTimes[AUTO_WIPER_CHECKS] = millis();
          doAutoWiperChecks();
          break;
        case AUTO_LIGHT_CHECKS:
          lastRunTimes[AUTO_LIGHT_CHECKS] = millis();
          doAutoLightChecks();
          break;
        case PROJECTABLE_HUD_UPDATES:
          lastRunTimes[PROJECTABLE_HUD_UPDATES] = millis();
//          doProjectableHUDUpdate();
          break;
        case BLINKING_INDICATORS:
          lastRunTimes[BLINKING_INDICATORS] = millis();
          doBlinkIndicators();
          break;
      }
    }
  }
}

void doGasCapacityMeasurement() {

  int resistance = 0;

    // read the input on analog pin 2:
  int sensorValue = analogRead(GAS_LEVEL);
  
  // print out the value you read:
  if (DEBUG) {
    say("Read voltage from A2 pin which represents gas level: %d bits", sensorValue);
  }

  gasLevelCurrentMeasurement[gasLevelCurrentMeasurementArrayPointer] = sensorValue;
  gasLevelCurrentMeasurementArrayPointer++;

  // Not enough measurements
  if (gasLevelCurrentMeasurementArrayPointer < AMOUNT_OF_MEASURES) {
    return;
  }

  // We've made enough measures to adjust digipot
  float gasLevelMean = calculateMean(gasLevelCurrentMeasurement, AMOUNT_OF_MEASURES);
  float gasLevelInVolts = ((gasLevelMean * 5) / 1024) + VOLTAGE_DEVIATION; // Also apply correction of voltage here

  if (DEBUG) {
    say("Read voltage from A2 pin which represents gas level in V: %f, mean was %f", gasLevelInVolts, gasLevelMean);
  }

  // Resetting all the counters.
  memset(gasLevelCurrentMeasurement, 0, AMOUNT_OF_MEASURES);
  gasLevelCurrentMeasurementArrayPointer = 0;

  // Due to curve for gas sensor is not linear we cannot use single formula to do conversion.
  // 5V - 3.12V (100 - 75%%)
  // 3.11V - 2.25V (74 - 50%%)
  // 2.24V - 1.17V (49 - 25 %%)
  // 1.16V - 1.09V (25 - 10%%)
  // 1.09V - 1.06V (smallest value is 1.06V) (10 - 0%%)

  if (gasLevelInVolts > 3.12) {
    resistance = round(((100 - ((((gasLevelInVolts - 3.12)*25)/1.88) + 75))/100)*MaxFuelGaugeResistance);
  } 

  if (gasLevelInVolts <= 3.12 && gasLevelInVolts > 2.25) {
    resistance = round(((100 - ((((gasLevelInVolts - 2.25)*24)/0.86) + 50))/100)*MaxFuelGaugeResistance);
  }

  if (gasLevelInVolts <= 2.25 && gasLevelInVolts > 1.17) {
    resistance = round(((100 - ((((gasLevelInVolts - 1.17)*24)/1.07) + 24))/100)*MaxFuelGaugeResistance);
  }

  if (gasLevelInVolts <= 1.17 && gasLevelInVolts > 1.09) {
    resistance = round(((100 - ((((gasLevelInVolts - 1.09)*14)/0.07) + 10))/100)*MaxFuelGaugeResistance);
  }

  if (gasLevelInVolts <= 1.09) {
    //resistance = round(((100 - (((gasLevelInVolts - 1.06)*10)/0.03))/100)*MaxFuelGaugeResistance);
    resistance = MaxFuelGaugeResistance;
  }

  // Detecting which step to set
  if (resistance > MaxFuelGaugeResistance) {
    resistance = MaxFuelGaugeResistance;
  }

  // Next loop will use new gauge resistance if it was changed here.
  adjustGaugeResistanceBasedOnSpentFuel(resistance);

  say("Setting step of digipot to: %d Ohms. Max resistance is: %d", resistance, MaxFuelGaugeResistance);
  setResistance(resistance);
}

void doAutoWiperChecks() {
  int autoWiperValue = analogRead(AUTO_WIPER);

  // print out the value you read:
  if (DEBUG) {
    say("Read voltage from Auto Wiper checks pin: %d bits", autoWiperValue);
  }


  // If current state matches button state - do not do anything
  if (
    autoWiperEnabled && autoWiperValue > 200 && !blinkingIndicators
    || !autoWiperEnabled && autoWiperValue <= 200
    ) {
    return;
  }

  if (autoWiperEnabled && autoWiperValue <= 200) {
    autoWiperEnabled = false;
    blinkingIndicators = false;
    say("Turning auto wiper OFF");
    digitalWrite(AUTO_WIPER_RELAY, LOW);
    digitalWrite(AUTO_WIPER_RELAY_INDICATOR, 0);
  }

  if (!autoWiperEnabled && autoWiperValue > 200) {
    autoWiperEnabled = true;
    say("Turning auto wiper ON");

    // To turn this on, we need to make sure that auto light wire is turned on as well, otherwise DDA-65 block will be unpowered.
    doAutoLightChecks();

    // Wait for relays to powerup and DDA-65 to boot up. However ignore it when system has just started, cause 
    // this is the way how the sensor is designed.
    if (millis() > 5000) {
      say("Switching blinking OFF");
      blinkingIndicators = false;
      if (!lightsAreOn) {
        doBlinkIndicators();
      }
      delay(1000);
    } else {
      blinkingIndicators = true;
      say("Both of switches are on, so blinking is enabled");
    }

    digitalWrite(AUTO_WIPER_RELAY, HIGH);
    analogWrite(AUTO_WIPER_RELAY_INDICATOR, INDICATOR_VOLTAGE);
  }
}
void doAutoLightChecks() {
  int autoLightValue = analogRead(AUTO_LIGHT);

  // print out the value you read:
  if (DEBUG) {
    say("Read voltage from Auto light checks pin: %d bits", autoLightValue);
  }


  // If current state matches button state - do not do anything
  // We also need to have relay on, when auto-wiper relay is on. This is coming from requirements of DDA-65 connection.
  if (
    autoLightEnabled && autoLightValue > 200
    || !autoLightEnabled && autoLightValue <= 200 && !autoWiperEnabled
    || autoLightEnabled && autoLightValue <= 200 && autoWiperEnabled // We need this wire to be on, otherwise DDA-65 will not consider block as powered
    ) {
    return;
  }

  if (autoLightEnabled && autoLightValue <= 200) {
    autoLightEnabled = false;
    say("Turning auto light sensor OFF.");
    digitalWrite(AUTO_LIGHT_RELAY, LOW);
    digitalWrite(AUTO_LIGHT_RELAY_INDICATOR, 0);
  }

  if (!autoLightEnabled && (autoLightValue > 200 || autoWiperEnabled)) {
    autoLightEnabled = true;
    say("Turning auto light sensor ON.");
    digitalWrite(AUTO_LIGHT_RELAY, HIGH);
    analogWrite(AUTO_LIGHT_RELAY_INDICATOR, round(INDICATOR_VOLTAGE*0.2)); // 80% of indicator value, cause otherwise it's too bright
  }

}
void doProjectableHUDUpdate() {
//    unsigned char stmp[8] = {4, 65, 13, 120, 224, 185, 147};
//   // // send data:  id = 0x00, standrad frame, data len = 1, stmp: data buf  
//      //int speed = random(120,150);
//      say("Seeting speed to: %d", stmp);
//      CAN.sendMsgBuf(0x00, 0, 7, stmp);
//   return;

// String BuildMessage="";


// unsigned char len = 0;
// unsigned char buf[8];
// char str[20];
//       char rndCoolantTemp=random(1,200);
//     char rndRPM=random(1,55);
//     char rndSpeed=random(0,255);
//     char rndIAT=random(0,255);
//     char rndMAF=random(0,255);
//     char rndAmbientAirTemp=random(0,200);
//     char rndCAT1Temp=random(1,55);
    
//     //GENERAL ROUTINE
//     unsigned char SupportedPID[8] =       {1,2,3,4,5,6,7,8};
//     unsigned char MilCleared[7] =         {4, 65, 63, 34, 224, 185, 147}; 
    
//     //SENSORS
//     unsigned char CoolantTemp[7] =        {4, 65, 5, rndCoolantTemp, 0, 185, 147};  
//     unsigned char rpm[7] =                {4, 65, 12, rndRPM, 224, 185, 147};
//     unsigned char vspeed[7] =             {4, 65, 13, rndSpeed, 224, 185, 147};
//     unsigned char IATSensor[7] =          {4, 65, 15, rndIAT, 0, 185, 147};
//     unsigned char MAFSensor[7] =          {4, 65, 16, rndMAF, 0, 185, 147};
//     unsigned char AmbientAirTemp[7] =     {4, 65, 70, rndAmbientAirTemp, 0, 185, 147};
//     unsigned char CAT1Temp[7] =           {4, 65, 60, rndCAT1Temp, 224, 185, 147};
//     unsigned char CAT2Temp[7] =           {4, 65, 61, rndCAT1Temp, 224, 185, 147};
//     unsigned char CAT3Temp[7] =           {4, 65, 62, rndCAT1Temp, 224, 185, 147};
//     unsigned char CAT4Temp[7] =           {4, 65, 63, rndCAT1Temp, 224, 185, 147};
    
//     if(CAN_MSGAVAIL == CAN.checkReceive())  
//     {
      
//       CAN.readMsgBuf(&len, buf); 
//         canId = CAN.getCanId();
//         Serial.print("<");Serial.print(canId);Serial.print(",");
//         for(int i = 0; i<len; i++)
//         {  
//           BuildMessage = BuildMessage + buf[i] + ",";
//         }
//         Serial.println(BuildMessage);
        
//         //Check wich message was received.
//         if(BuildMessage=="2,1,0,0,0,0,0,0,") {CAN.sendMsgBuf(0x7E8, 0, 8, SupportedPID);}
//         if(BuildMessage=="2,1,1,0,0,0,0,0,") {CAN.sendMsgBuf(0x7E8, 0, 7, MilCleared);}
        
//         //SEND SENSOR STATUSES
//         if(BuildMessage=="2,1,5,0,0,0,0,0,") {CAN.sendMsgBuf(0x7E8, 0, 7, CoolantTemp);}
//         if(BuildMessage=="2,1,12,0,0,0,0,0,"){CAN.sendMsgBuf(0x7E8, 0, 7, rpm);}
//         if(BuildMessage=="2,1,13,0,0,0,0,0,"){CAN.sendMsgBuf(0x7E8, 0, 7, vspeed);}
//         if(BuildMessage=="2,1,15,0,0,0,0,0,"){CAN.sendMsgBuf(0x7E8, 0, 7, IATSensor);}
//         if(BuildMessage=="2,1,16,0,0,0,0,0,"){CAN.sendMsgBuf(0x7E8, 0, 7, MAFSensor);}
//         if(BuildMessage=="2,1,70,0,0,0,0,0,"){CAN.sendMsgBuf(0x7E8, 0, 7, AmbientAirTemp);}
//         if(BuildMessage=="2,1,60,0,0,0,0,0,"){CAN.sendMsgBuf(0x7E8, 0, 7, CAT1Temp);}
//         if(BuildMessage=="2,1,61,0,0,0,0,0,"){CAN.sendMsgBuf(0x7E8, 0, 7, CAT2Temp);}
//         if(BuildMessage=="2,1,62,0,0,0,0,0,"){CAN.sendMsgBuf(0x7E8, 0, 7, CAT3Temp);}
//         if(BuildMessage=="2,1,63,0,0,0,0,0,"){CAN.sendMsgBuf(0x7E8, 0, 7, CAT4Temp);}
//         BuildMessage="";
//     }
}

void doBlinkIndicators() {
  if (lightsAreOn) {
    analogWrite(AUTO_WIPER_RELAY_INDICATOR, 0);
    analogWrite(AUTO_LIGHT_RELAY_INDICATOR, 0);
    lightsAreOn = false;
  } else {
    analogWrite(AUTO_WIPER_RELAY_INDICATOR, INDICATOR_VOLTAGE);
    analogWrite(AUTO_LIGHT_RELAY_INDICATOR, round(INDICATOR_VOLTAGE*0.2)); // 80% of indicator value, cause otherwise it's too bright
    lightsAreOn = true;
  }
}

// My instance of AD5206 has damaged 2nd channel. So I had to adjust code for this.
void setResistance(int resistance) {

  // Adding correction of resistance here, however only after we reach less than 25% of tank capacity, otherwise
  // fuel levels are quickly decreasing in beginning and then very slowly in the end.
  resistanceCorrection = 0;
  if (autoLightEnabled && resistance > round(MaxFuelGaugeResistance*0.75)) {
    resistanceCorrection += round(resistance*RESISTANCE_CORRECTION_STEP);
  }

  if (autoWiperEnabled && resistance > round(MaxFuelGaugeResistance*0.75)) {
    resistanceCorrection += round(MaxFuelGaugeResistance*RESISTANCE_CORRECTION_STEP);
  }

  resistance += resistanceCorrection;

  step = round((resistance - DIGIPOT_MINIMAL_RESISTANCE) / STEP_RESISTANCE);

  if (step < 0) {
    step = 0;
  }

  // If resistance is already set to the same step - do not change anything
  if (gasLevelResistanceStepSetPreviously == step) {
    return;
  }
  
  gasLevelResistanceStepSetPreviously = step;
  int levelPerChannel = floor(step/(NUMBER_OF_CHANNELS_IN_DIGIPOT - NUMBER_OF_DEFECTIVE_CHANNELS_IN_DIGIPOT));
  int leftOvers = step - levelPerChannel*(NUMBER_OF_CHANNELS_IN_DIGIPOT - NUMBER_OF_DEFECTIVE_CHANNELS_IN_DIGIPOT);
  if (DEBUG) {
    say("Level per channel identified: %d. Leftovers: %d. Resistance is set to: %d", levelPerChannel, leftOvers, resistance);
  }  
  for (int channel = 0; channel < NUMBER_OF_CHANNELS_IN_DIGIPOT; channel++) {

    // Skip setting defective channel values
    if (channel == 2) {
      continue;
    }
    
    if (leftOvers) {
      digitalPotWrite(channel, levelPerChannel + 1);
      if (DEBUG) {
        say("Setting %d channel to step %d", channel, levelPerChannel + 1);
      }
      leftOvers--;
    } else {
      digitalPotWrite(channel, levelPerChannel);
      if (DEBUG) {
        say("Setting %d channel to step %d", channel, levelPerChannel);
      }
    }
  }
}

void updateGasValveOpen() {

  // Do initial reading. If it's 0 (for example gas equipment is off), do not run through other cycles
  int currentValveState = analogRead(GAS_VALVE);
  if (DEBUG) {
    currentValveState = 500;
  }

  if (DEBUG) {
    say("Read voltage from A3 pin which represents gas valve state: %d bits", currentValveState);
  }

  // 200 is around 1V of voltage
  if (currentValveState < 200) {
    setDashboardSource(false);

    // If valve state is closed, then we do not need to continue measurement. Resetting all the counters.
    taskCooldowns[GAS_VALVE_CHECKS] = GAS_VALVE_CHECKS_LOW_FREQUENCY_CHECKS;
    memset(gasValveCurrentMeasurement, 0, AMOUNT_OF_MEASURES);
    gasValveCurrentMeasurementArrayPointer = 0;

    return;
  }

  // If valve state is open, let's measure how open it is with series of checks.
  taskCooldowns[GAS_VALVE_CHECKS] = GAS_VALVE_CHECKS_HIGH_FREQUENCY_CHECKS;

  gasValveCurrentMeasurement[gasValveCurrentMeasurementArrayPointer] = currentValveState;
  gasValveCurrentMeasurementArrayPointer++;

  // We've made enough measures
  if (gasValveCurrentMeasurementArrayPointer == AMOUNT_OF_MEASURES) {
    float standardDeviation = calculateSD(gasValveCurrentMeasurement, AMOUNT_OF_MEASURES);

    if (DEBUG) {
      say("Standard deviation: %f", standardDeviation);
    }

    if (standardDeviation < 40) {
      setDashboardSource(true);
    } else {
      setDashboardSource(false);
    }

    // Resetting all the counters.
    taskCooldowns[GAS_VALVE_CHECKS] = GAS_VALVE_CHECKS_LOW_FREQUENCY_CHECKS;
    memset(gasValveCurrentMeasurement, 0, AMOUNT_OF_MEASURES);
    gasValveCurrentMeasurementArrayPointer = 0;
  }

  return;
}

void setDashboardSource(bool state) {
  if (state == isGasValveOpen) {
    return;
  }

  if (DEBUG) {
    say("Entering function of setting fuel measurement source with request to set state %d", state);
  }

  if (state) {
    digitalWrite(FUEL_DASHBOARD_RELAY, HIGH);
  } else {
    digitalWrite(FUEL_DASHBOARD_RELAY, LOW);
  }

  isGasValveOpen = state;

  return;
}

// This function switches gauge resistance based on consumed fuel since last boot of device.
// If HIGHWAY_RESISTANCE_THRESHOLD of fuel consumed during last device boot - we're on the highway
void adjustGaugeResistanceBasedOnSpentFuel(int resistance) {

  // Probably inactive gas installation or too early to measure
  if (
    resistance <= 0 // inactive gas installation
    || millis() <= 20000 // too early to measure
    || MaxFuelGaugeResistance == MAX_FUEL_GAUGE_RESISTANCE_HIGHWAY // mode already set
    ) {
    return;
  }

  // We set gauge resistance since start after 20 seconds of device boot
  if (!gaugeResistanceSinceStart && millis() > 20000) {
    gaugeResistanceSinceStart = resistance;
    if (DEBUG) {
      say("Setting resistance since start is %d, while resistance is %d", gaugeResistanceSinceStart, resistance);
    }
    return;
  }

  if ((gaugeResistanceSinceStart - resistance) >= MaxFuelGaugeResistance*HIGHWAY_RESISTANCE_THRESHOLD) {
    MaxFuelGaugeResistance = MAX_FUEL_GAUGE_RESISTANCE_HIGHWAY;
    say("Highway pattern of fuel use detected: was: %d now: %d. Setting maximum gauge resistance to %d Ohm", gaugeResistanceSinceStart, resistance, MaxFuelGaugeResistance);
    return;
  }
}

/** 
 *  System functions
 *  
 */


#ifndef SAY
#define SAY
#define ARDBUFFER 16

int say(char *str, ...)
{
  int i, count=0, j=0, flag=0;
  char temp[ARDBUFFER+1];
  for(i=0; str[i]!='\0';i++)  if(str[i]=='%')  count++;

  va_list argv;
  va_start(argv, count);
  for(i=0,j=0; str[i]!='\0';i++)
  {
    if(str[i]=='%')
    {
      temp[j] = '\0';
      Serial.print(temp);
      j=0;
      temp[0] = '\0';

      switch(str[++i])
      {
        case 'd': Serial.print(va_arg(argv, int));
                  break;
        case 'l': Serial.print(va_arg(argv, long));
                  break;
        case 'f': Serial.print(va_arg(argv, double));
                  break;
        case 'c': Serial.print((char)va_arg(argv, int));
                  break;
        case 's': Serial.print(va_arg(argv, char *));
                  break;
        default:  ;
      };
    }
    else 
    {
      temp[j] = str[i];
      j = (j+1)%ARDBUFFER;
      if(j==0) 
      {
        temp[ARDBUFFER] = '\0';
        Serial.print(temp);
        temp[0]='\0';
      }
    }
  };
  Serial.println();
  return count + 1;
}
#undef ARDBUFFER
#endif

void digitalPotWrite(int address, int value) {
  // take the SS pin low to select the chip:
  digitalWrite(slaveSelectPin, LOW);
  //  send in the address and value via SPI:
  SPI.transfer(address);
  SPI.transfer(value);
  // take the SS pin high to de-select the chip:
  digitalWrite(slaveSelectPin, HIGH);
}


float calculateSD(int data[], int sizeOfArray) {
  float sum = 0.0, mean, standardDeviation = 0.0;
  for(int i=0;i<sizeOfArray;++i) {
    sum += data[i];
  }

  mean = sum/sizeOfArray;

  for(int i=0;i<sizeOfArray;++i) {
    standardDeviation += pow(data[i] - mean, 2);
  }

  return sqrt(standardDeviation / sizeOfArray);

}

float calculateMean(int data[], int sizeOfArray) {
  float sum = 0.0, mean;
  for(int i=0;i<sizeOfArray;++i) {
    sum += data[i];
  }

  mean = sum/sizeOfArray;
  return mean;
}
