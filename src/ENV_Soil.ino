/*
* Project : Environment monitoring with Soil Moisture Sensor. 
* Description: Cellular Connected Data Logger for monitoring environment.
      * Sensors : 
          SHT31 Temperature and Humidity Sensor
          Adafruit VEML7700 Light Sensor
          MB85RC64 FRAM 
          MCP79410 Real time clock 
          Soil Moisture sensor.
      * Solar Powered with USB Solar Panel.
      * Reporting Frequency 15 Minutes ( If low power, frequency = 30 minutes.)
      * Pinouts : 
          Watchdog : D8 and D5

* Author: Abdul Hannan Mustajab
* Latest Revision: 19th July 2022
*/



// v1.00- ENV Code as starting point. 
// v1.01 - Added soil moisture sensor
// v1.20 - Added second soil moisture sensor to the board. 
// v1.21 - Fixed issue with the size of the data payload. 



#define SOFTWARERELEASENUMBER "1.21"                                                        // Keep track of release numbers

// Included Libraries
#include "math.h"
#include "adafruit-sht31.h"
#include "Adafruit_VEML7700.h"
#include "PublishQueueAsyncRK.h"                                                            // Async Particle Publish
#include "MB85RC256V-FRAM-RK.h"                                                             // Rickkas Particle based FRAM Library
#include "MCP79410RK.h"   

// Define the memory map - note can be EEPROM or FRAM - moving to FRAM for speed and to avoid memory wear
namespace FRAM {                                                                         // Moved to namespace instead of #define to limit scope
  enum Addresses {
    versionAddr           = 0x00,                                                           // Where we store the memory map version number - 8 Bits
    sysStatusAddr         = 0x01,                                                           // This is the status of the device
    sensorDataAddr        = 0xA0                                                            // Where we store the latest sensor data readings
   };
};

const int FRAMversionNumber = 5;                                                            // Increment this number each time the memory map is changed

struct systemStatus_structure {                     
  uint8_t structuresVersion;                                                                // Version of the data structures (system and data)
  bool thirdPartySim;                                                                       // If this is set to "true" then the keep alive code will be executed
  int keepAlive;                                                                            // Keep alive value for use with 3rd part SIMs
  bool connectedStatus;
  bool verboseMode;
  bool lowBatteryMode;
  bool lowPowerMode;
  int stateOfCharge;                                                                        // Battery charge level
  uint8_t batteryState;                                                                     // Stores the current battery state
  int resetCount;                                                                           // reset count of device (0-256)
  unsigned long lastHookResponse;                                                           // Last time we got a valid Webhook response
} sysStatus;


// Keypad struct for mapping buttons, notes, note values, LED array index, and default color
struct sensor_data_struct {                                                               // Here we define the structure for collecting and storing data from the sensors
  bool validData;
  unsigned long timeStamp;
  float batteryVoltage;
  float temperatureInC;
  float relativeHumidity;
  float lux;
  float white;
  float raw_als;   
  int stateOfCharge;
  int powerSource;
  int soilMoisture; 
  int soilMoisture2;

} sensor_data;

// Prototypes and System Mode calls
SYSTEM_MODE(SEMI_AUTOMATIC);                                                                     // This will enable user code to start executing automatically.
SYSTEM_THREAD(ENABLED);                                                                     // Means my code will not be held up by Particle processes.
STARTUP(System.enableFeature(FEATURE_RESET_INFO));
SystemSleepConfiguration config;                                                            // Initialize new Sleep 2.0 Api

Adafruit_VEML7700 veml;
Adafruit_SHT31 sht31 = Adafruit_SHT31();
MB85RC64 fram(Wire, 0);                                                                     // Rickkas' FRAM library
MCP79410 rtc;                                                                               // Rickkas MCP79410 libarary
retained uint8_t publishQueueRetainedBuffer[2048];                                          // Create a buffer in FRAM for cached publishes
PublishQueueAsync publishQueue(publishQueueRetainedBuffer, sizeof(publishQueueRetainedBuffer));
Timer keepAliveTimer(1000, keepAliveMessage);

// State Machine Variables
enum State { INITIALIZATION_STATE, ERROR_STATE, IDLE_STATE, MEASURING_STATE, REPORTING_STATE, RESP_WAIT_STATE,NAPPING_STATE,SLEEPING_STATE};
char stateNames[8][26] = {"Initialize", "Error", "Idle", "Measuring","Reporting", "Response Wait","Napping State","Sleeping State"};
State state = INITIALIZATION_STATE;
State oldState = INITIALIZATION_STATE;

// Pin Constants - Boron Carrier Board v1.2a
const int wakeUpPin = D8;                                                                   // This is the Particle Electron WKP pin
const int donePin = D5;
const int userSwitch =    D4;                                                               // User switch with a pull-up resistor
const int soilPin =      A0;                                                                //  Soil Sensor
const int soilPin2 =      A1;                                                               // Second Soil Sensor


volatile bool watchdogFlag=false;                                                           // Flag to let us know we need to pet the dog


// Timing Variables
const unsigned long webhookWait = 45000;                                                    // How long will we wair for a WebHook response
const unsigned long resetWait   = 300000;                                                   // How long will we wait in ERROR_STATE until reset
unsigned long stayAwakeTimeStamp = 0;
unsigned long stayAwake;                                                                    // Stores the time we need to wait before napping
const unsigned long stayAwakeLong = 90000;                                                  // In lowPowerMode, how long to stay awake every hour


unsigned long webhookTimeStamp  = 0;                                                        // Webhooks...
unsigned long resetTimeStamp    = 0;                                                        // Resets - this keeps you from falling into a reset loop
bool dataInFlight = false;
bool flashTheLED = false;                                                                   // Flag that will flash the LED if any thresholds are crossed


// Timing Variables

int publishInterval;                                                                        // Publish interval for sending data. 
const int publishFrequency      = 1000;                                                     // We can only publish once a second
int sampleRate;                                                                             // Sample rate for idle state.
time_t t;     

// Variables Related To Particle Mobile Application Reporting
// Simplifies reading values in the Particle Mobile Application
char temperatureString[16];
char humidityString[16];
char batteryContextStr[16];                                                                 // One word that describes whether the device is getting power, charging, discharging or too cold to charge
char batteryString[16];
char luxString[16];                                                                         // String to show the current threshold readings.                         
char whiteString[16];                                                                         // String to show the current threshold readings.                         
char ALSString[16];                                                                         // String to show the current threshold readings.                         
char lowPowerModeStr[6];                            // In low power mode?
bool systemStatusWriteNeeded = false;                                                          // Keep track of when we need to write
bool alertsStatusWriteNeeded = false;         
bool sensorDataWriteNeeded = false; 


// Time Period Related Variables
const int wakeBoundary = 0*3600 + 20*60 + 0;                                                // 0 hour 20 minutes 0 seconds
const int keepAliveBoundary = 0*3600 + 5*60 +0;                                             // How often do we need to send a ping to keep the connection alive - start with 5 minutes - *** related to keepAlive value in Setup()! ***


// Program Variables
const char* releaseNumber = SOFTWARERELEASENUMBER;                                          // Displays the release on the menu


#define MEMORYMAPVERSION 2                          // Lets us know if we need to reinitialize the memory map

void setup()                                                                                // Note: Disconnected Setup()
{
  pinMode(wakeUpPin,INPUT);                                                                 // This pin is active HIGH, 
  pinMode(donePin,OUTPUT);                                                                  // Allows us to pet the watchdog
  pinMode(userSwitch,INPUT);                                                                // Momentary contact button on board for direct user input
  pinMode(soilPin, INPUT);    

  petWatchdog();                                                                           // Pet the watchdog - This will reset the watchdog time period AND 
  attachInterrupt(wakeUpPin, watchdogISR, RISING);                                         // The watchdog timer will signal us and we have to respond

  char StartupMessage[64] = "Startup Successful";                                           // Messages from Initialization
  state = IDLE_STATE;

  Particle.subscribe(System.deviceID() + "/hook-response/environmental-hook/", UbidotsHandler, MY_DEVICES);

  Particle.variable("Release",releaseNumber);
  Particle.variable("temperature", temperatureString);
  Particle.variable("humidity", humidityString);
  Particle.variable("Lux", luxString);
  Particle.variable("White", whiteString);
  Particle.variable("Raw ALS", ALSString);
  Particle.variable("Battery", batteryString);                                              // Battery level in V as the Argon does not have a fuel cell
  Particle.variable("BatteryContext",batteryContextStr);
  Particle.variable("Keep Alive Sec",sysStatus.keepAlive);
  Particle.variable("3rd Party Sim", sysStatus.thirdPartySim);
  Particle.variable("lowPowerMode",lowPowerModeStr);
  
  
  Particle.function("Measure-Now",measureNow);
  Particle.function("Verbose-Mode",setVerboseMode);
  Particle.function("Keep Alive",setKeepAlive);
  Particle.function("3rd Party Sim", setThirdPartySim);
  Particle.function("Set Low Power",setLowPowerMode);


  rtc.setup();                                                                            // Start the real time clock
  rtc.clearAlarm();                                                                       // Ensures alarm is still not set from last cycle

  if (! sht31.begin(0x44)) {                                                                      // Start the BME680 Sensor
    snprintf(StartupMessage,sizeof(StartupMessage),"Error - SHT31 Initialization");
    state = ERROR_STATE;
    resetTimeStamp = millis();
  }

  if (!veml.begin()) {                                                                      // Start the BME680 Sensor
    resetTimeStamp = millis();
    snprintf(StartupMessage,sizeof(StartupMessage),"Error - VEML Initialization");
    state = ERROR_STATE;
    resetTimeStamp = millis();
  }

  veml.setGain(VEML7700_GAIN_1/8); 
  veml.setIntegrationTime(VEML7700_IT_25MS);

  veml.interruptEnable(true);

  // Load FRAM and reset variables to their correct values
  fram.begin();                                                                             // Initialize the FRAM module
  
  byte tempVersion;
  fram.get(FRAM::versionAddr, tempVersion);
  if (tempVersion != FRAMversionNumber) {                                                   // Check to see if the memory map in the sketch matches the data on the chip
    fram.erase();                                                                           // Reset the FRAM to correct the issue
    fram.put(FRAM::versionAddr, FRAMversionNumber);                                         // Put the right value in
    fram.get(FRAM::versionAddr, tempVersion);                                               // See if this worked
    if (tempVersion != FRAMversionNumber) state = ERROR_STATE;                              // Device will not work without FRAM
    else {
      loadSystemDefaults();                                                                 // Out of the box, we need the device to be awake and connected
    }
  }
  else {
    fram.get(FRAM::sysStatusAddr,sysStatus);                                                // Loads the System Status array from FRAM
  }

  checkSystemValues();                                                                      // Make sure System values are all in valid range

  connectToParticle();

  (sysStatus.lowPowerMode) ? strncpy(lowPowerModeStr,"True",sizeof(lowPowerModeStr)) : strncpy(lowPowerModeStr,"False",sizeof(lowPowerModeStr));


  if (sysStatus.thirdPartySim) {
    waitUntil(Particle.connected); 
    Particle.keepAlive(sysStatus.keepAlive);                                                    // Set the keep alive value
    keepAliveTimer.changePeriod(sysStatus.keepAlive*1000);                                  // Will start the repeating timer
  }
  
  if (!digitalRead(userSwitch)) loadSystemDefaults();                                       // Make sure the device wakes up and connects
  
  takeMeasurements();                                                                       // For the benefit of monitoring the device
  
  if (sysStatus.lowBatteryMode) setLowPowerMode("1");                                       // If battery is low we need to go to low power state

  if(sysStatus.verboseMode) publishQueue.publish("Startup",StartupMessage,PRIVATE);                       // Let Particle know how the startup process went

  if (state == INITIALIZATION_STATE) state = IDLE_STATE;                                    // We made it throughgo let's go to idle
}

void loop()
{

  switch(state) {
  
  case IDLE_STATE:
    if (sysStatus.verboseMode && state != oldState) publishStateTransition();
    if (sysStatus.lowPowerMode && (millis() - stayAwakeTimeStamp) > stayAwake) state = NAPPING_STATE; 
    // if (sysStatus.lowBatteryMode) state = SLEEPING_STATE;                                 // If in low power mode, Goto sleep.
    if (!(Time.now() % wakeBoundary)) state = MEASURING_STATE;  
    
    break;

  case MEASURING_STATE:                                                                     // Take measurements prior to sending
    if (sysStatus.verboseMode && state != oldState) publishStateTransition();
    if (!takeMeasurements())
    {
      state = ERROR_STATE;
      resetTimeStamp = millis();
      if (sysStatus.verboseMode) {
        publishQueue.publish("State","Error taking Measurements",PRIVATE);
      }
    }
    else state = REPORTING_STATE;
    break;

  case REPORTING_STATE:
    if (sysStatus.verboseMode && state != oldState) publishStateTransition();                         // Reporting - hourly or on command
    if (!sysStatus.connectedStatus) connectToParticle();                                              // Only attempt to connect if not already New process to get connected
    if (Particle.connected()) {
      if (Time.hour() == 12) Particle.syncTime();                                                     // Set the clock each day at noon
      sendEvent();                                                                                    // Send data to Ubidots
      state = RESP_WAIT_STATE;                                                                        // Wait for Response
    }
    else {
      state = ERROR_STATE;
      resetTimeStamp = millis();
    }
    break;

  case RESP_WAIT_STATE:
    if (sysStatus.verboseMode && state != oldState) publishStateTransition();
    if (!dataInFlight)                                                // Response received back to IDLE state
    {
     state = IDLE_STATE;
     stayAwake = stayAwakeLong;
     stayAwakeTimeStamp = millis();
    }
    else if (millis() - webhookTimeStamp > webhookWait) {             // If it takes too long - will need to reset
      resetTimeStamp = millis();
      publishQueue.publish("spark/device/session/end", "", PRIVATE);      // If the device times out on the Webhook response, it will ensure a new session is started on next connect
      state = ERROR_STATE;                                            // Response timed out
      resetTimeStamp = millis();
    }
    break;

  
  case ERROR_STATE:                                                                         // To be enhanced - where we deal with errors
    if (sysStatus.verboseMode && state != oldState) publishStateTransition();
    if (millis() > resetTimeStamp + resetWait)
    {
      if (Particle.connected()) publishQueue.publish("State","Error State - Reset", PRIVATE);    // Brodcast Reset Action
      delay(2000);
      System.reset();
    }
    break;

  case NAPPING_STATE: { // This state puts the device to sleep mode
      if (sysStatus.verboseMode && oldState != state) publishStateTransition();                    // If verboseMode is on and state is changed, Then publish the state transition.
      if (sysStatus.connectedStatus) disconnectFromParticle();          // Disconnect cleanly from Particle
      stayAwake = 1000;                                                 // Once we come into this function, we nee  d to reset stayAwake as it changes at the top of the hour
      int wakeInSeconds = constrain(wakeBoundary - Time.now() % wakeBoundary, 1, wakeBoundary);
      config.mode(SystemSleepMode::ULTRA_LOW_POWER)
      .gpio(userSwitch,CHANGE)
      .duration(wakeInSeconds*1000);
      SystemSleepResult result = System.sleep(config);

      if (result.wakeupPin() == userSwitch) {                            // If the user woke the device we need to get up
        setLowPowerMode("0");
      }
      state = MEASURING_STATE;
    } break; 

  case SLEEPING_STATE: {                                              // This state is triggered when the device is in Low Power mode. 
      if (sysStatus.verboseMode && state != oldState) publishStateTransition();
      if (sysStatus.connectedStatus) disconnectFromParticle();          // Disconnect cleanly from Particle
      
      petWatchdog();
      int wakeInSeconds = constrain(wakeBoundary - Time.now() % wakeBoundary, 1, wakeBoundary);
      config.mode(SystemSleepMode::ULTRA_LOW_POWER)
        .gpio(userSwitch,CHANGE)
        .duration(wakeInSeconds*1000);
        System.sleep(config);

      SystemSleepResult result = System.sleep(config);                   // Put the device to sleep device reboots from here   
      if (result.wakeupPin() == userSwitch) {                            // If the user woke the device we need to get up
        setLowPowerMode("0");
      }
      state = IDLE_STATE;                                               //  Head back to the idle state to see what to do next
    } break;
  
  }

  rtc.loop();                                                                               // keeps the clock up to date

  if (watchdogFlag) petWatchdog();                                                          // Watchdog flag is raised - time to pet the watchdog

  if (systemStatusWriteNeeded) {
    fram.put(FRAM::sysStatusAddr,sysStatus);
    systemStatusWriteNeeded = false;
  }
  if (sensorDataWriteNeeded) {
    fram.put(FRAM::sensorDataAddr,sensor_data);
    sensorDataWriteNeeded = false;
  }
}


void loadSystemDefaults() {                                                                 // Default settings for the device - connected, not-low power and always on
  if (Particle.connected()) publishQueue.publish("Mode","Loading System Defaults", PRIVATE);
  sysStatus.thirdPartySim = 0;
  sysStatus.keepAlive = 120;
  sysStatus.structuresVersion = 1;
  sysStatus.verboseMode = false;
  sysStatus.lowBatteryMode = false;
  fram.put(FRAM::sysStatusAddr,sysStatus);                                                  // Write it now since this is a big deal and I don't want values over written
}


void checkSystemValues() {                                          // Checks to ensure that all system values are in reasonable range 
  if (sysStatus.keepAlive < 0 || sysStatus.keepAlive > 1200) sysStatus.keepAlive = 600;
  if (sysStatus.resetCount < 0 || sysStatus.resetCount > 255) sysStatus.resetCount = 0;
  systemStatusWriteNeeded = true;
}


void watchdogISR()
{
  watchdogFlag = true;
}
void petWatchdog()
{
  digitalWriteFast(donePin, HIGH);                                        // Pet the watchdog
  digitalWriteFast(donePin, LOW);
  watchdogFlag = false;
}


void sendEvent()
{
  char data[256];                   
  snprintf(data, sizeof(data), "{\"temperature\":%4.1f,  \"humidity\":%4.1f,  \"Soilmoisture\":%i,  \"Soilmoisture2\":%i,  \"lux\":%4.1f,  \"white\":%4.1f,  \"als\":%4.1f,\"battery\":%i}", sensor_data.temperatureInC, sensor_data.relativeHumidity,sensor_data.soilMoisture, sensor_data.soilMoisture2 ,sensor_data.lux,sensor_data.white,sensor_data.raw_als,sensor_data.stateOfCharge);
  publishQueue.publish("environmental-hook", data, PRIVATE);
  dataInFlight = true;                                                                      // set the data inflight flag
  webhookTimeStamp = millis();
}

void UbidotsHandler(const char *event, const char *data) {            // Looks at the response from Ubidots - Will reset Photon if no successful response
  char responseString[64];
    // Response is only a single number thanks to Template
  if (!strlen(data)) {                                                // No data in response - Error
    snprintf(responseString, sizeof(responseString),"No Data");
  }
  else if (atoi(data) == 200 || atoi(data) == 201) {
    snprintf(responseString, sizeof(responseString),"Response Received");
    sysStatus.lastHookResponse = Time.now();                          // Record the last successful Webhook Response
    systemStatusWriteNeeded = true;
    dataInFlight = false;                                             // Data has been received
  }
  else {
    snprintf(responseString, sizeof(responseString), "Unknown response recevied %i",atoi(data));
  }
  publishQueue.publish("Ubidots Hook", responseString, PRIVATE);
}

// These are the functions that are part of the takeMeasurements call

bool takeMeasurements() {
  
  sensor_data.validData = false;

  if (sht31.readTemperature()){
  
    // Read and add temperature to the sensor data object.
    sensor_data.temperatureInC = sht31.readTemperature();
    snprintf(temperatureString,sizeof(temperatureString),"%4.1f*C", sensor_data.temperatureInC);

    // Read and add humidity to the sensor data object.
    sensor_data.relativeHumidity = sht31.readHumidity();
    snprintf(humidityString,sizeof(humidityString),"%4.1f%%", sensor_data.relativeHumidity);

    // Read and add lux,white and raw value to the sensor data object.
    sensor_data.lux = veml.readLux();
    sensor_data.white = veml.readWhite();
    sensor_data.raw_als = veml.readALS();

    // Read soil moisture 
    sensor_data.soilMoisture = map(analogRead(soilPin),0,3722,0,100);             // Sensor puts out 0-3V for 0% to 100% soil moisuture

    // Read soil moisture 
    sensor_data.soilMoisture2 = map(analogRead(soilPin2),0,3722,0,100);             // Sensor puts out 0-3V for 0% to 100% soil moisuture


    snprintf(luxString,sizeof(luxString),"Lux : %4.1f", sensor_data.lux);
    snprintf(whiteString,sizeof(whiteString),"White : %4.1f", sensor_data.white);
    snprintf(ALSString,sizeof(ALSString),"ALS : %4.1f", sensor_data.raw_als);

    sensor_data.stateOfCharge = int(System.batteryCharge());
    snprintf(batteryString, sizeof(batteryString), "%i %%", sensor_data.stateOfCharge);
   
    getBatteryContext();                   // Check what the battery is doing.

    sysStatus.stateOfCharge = int(System.batteryCharge());              // Percentage of full charge
    if (sysStatus.stateOfCharge < 30) {
      sysStatus.lowBatteryMode = true;
      if (!sysStatus.lowPowerMode) setLowPowerMode("1");                 // Should be there already but just in case...
      }  // Check to see if we are in low battery territory
    else sysStatus.lowBatteryMode = false;                              // We have sufficient to continue operations

    // Indicate that this is a valid data array and store it
    sensor_data.validData = true;
    sensor_data.timeStamp = Time.now();
    sensorDataWriteNeeded = true;  
    return 1;

    }else return 0;
  }                                                                       // Take measurement from all the sensors
 




// These are the particle functions that allow you to configure and run the device
// They are intended to allow for customization and control during installations
// and to allow for management.


int measureNow(String command) // Function to force sending data in current hour
{
  if (command == "1") {
    state = MEASURING_STATE;
    return 1;
  }
  else return 0;
}

int setVerboseMode(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    sysStatus.verboseMode = true;
    publishQueue.publish("Mode","Set Verbose Mode",PRIVATE);
    systemStatusWriteNeeded = true;
    return 1;
  }
  else if (command == "0")
  {
    sysStatus.verboseMode = false;
    publishQueue.publish("Mode","Cleared Verbose Mode",PRIVATE);
    systemStatusWriteNeeded = true;
    return 1;
  }
  else return 0;
}


void publishStateTransition(void)
{
  char stateTransitionString[40];
  snprintf(stateTransitionString, sizeof(stateTransitionString), "From %s to %s", stateNames[oldState],stateNames[state]);
  oldState = state;
  if(Particle.connected()) publishQueue.publish("State Transition",stateTransitionString, PRIVATE);
}



int setThirdPartySim(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    sysStatus.thirdPartySim = true;
    Particle.keepAlive(sysStatus.keepAlive);                                                // Set the keep alive value
    keepAliveTimer.changePeriod(sysStatus.keepAlive*1000);                                  // Will start the repeating timer
    if (Particle.connected()) publishQueue.publish("Mode","Set to 3rd Party Sim", PRIVATE);
    systemStatusWriteNeeded = true;
    return 1;
  }
  else if (command == "0")
  {
    sysStatus.thirdPartySim = false;
    if (Particle.connected()) publishQueue.publish("Mode","Set to Particle Sim", PRIVATE);
    systemStatusWriteNeeded = true;
    return 1;
  }
  else return 0;
}


int setKeepAlive(String command)
{
  char * pEND;
  char data[256];
  int tempTime = strtol(command,&pEND,10);                                                  // Looks for the first integer and interprets it
  if ((tempTime < 0) || (tempTime > 1200)) return 0;                                        // Make sure it falls in a valid range or send a "fail" result
  sysStatus.keepAlive = tempTime;
  if (Particle.connected()) {
    snprintf(data, sizeof(data), "Keep Alive set to %i sec",sysStatus.keepAlive);
    publishQueue.publish("Keep Alive",data, PRIVATE);
  }
  systemStatusWriteNeeded = true;                                                           // Need to store to FRAM back in the main loop
  return 1;
}

/**
 * @brief Toggles the device into low power mode based on the input command.
 * 
 * @details If the command is "1", sets the device into low power mode. If the command is "0",
 * sets the device into normal mode. Fails if neither of these are the inputs.
 *
 * @param command A string indicating whether to set the device into low power mode or into normal mode.
 * A "1" indicates low power mode, a "0" indicates normal mode. Inputs that are neither of these commands
 * will cause the function to return 0 to indicate an invalid entry.
 * 
 * @return 1 if able to successfully take action, 0 if invalid command
 */
int setLowPowerMode(String command)                                   // This is where we can put the device into low power mode if needed
{
  if (command != "1" && command != "0") return 0;                     // Before we begin, let's make sure we have a valid input
  if (command == "1")                                                 // Command calls for setting lowPowerMode
  {
    if (Particle.connected()) {
      publishQueue.publish("Mode","Low Power Mode", PRIVATE);
    }
    sysStatus.lowPowerMode = true;
    strncpy(lowPowerModeStr,"True", sizeof(lowPowerModeStr));
  }
  else if (command == "0")                                            // Command calls for clearing lowPowerMode
  {
    if (!Particle.connected()) {                                      // In case we are not connected, we will do so now.
      connectToParticle();
      sysStatus.connectedStatus = true;
    }
    publishQueue.publish("Mode","Normal Operations", PRIVATE);
    delay(1000);                                                      // Need to make sure the message gets out.
    sysStatus.lowPowerMode = false;                                   // update the variable used for console status
    strncpy(lowPowerModeStr,"False", sizeof(lowPowerModeStr));                                  // Use capitalization so we know that we set this.
  }
  systemStatusWriteNeeded = true;
  return 1;
}


void getBatteryContext() {
  const char* batteryContext[7] ={"Unknown","Not Charging","Charging","Charged","Discharging","Fault","Diconnected"};
  // Battery conect information - https://docs.particle.io/reference/device-os/firmware/boron/#batterystate-

  snprintf(batteryContextStr, sizeof(batteryContextStr),"%s", batteryContext[System.batteryState()]);

}

void keepAliveMessage() {
  publishQueue.publish("*", PRIVATE,NO_ACK);
}

/*

*/


bool connectToParticle() {
  Cellular.on();
  Particle.connect();
  // wait for *up to* 5 minutes
  for (int retry = 0; retry < 300 && !waitFor(Particle.connected,1000); retry++) {
    Particle.process();                                           // Keeps the device responsive as it is not traversing the main loop
    petWatchdog();                                                // Pet the watchdog as we are out of the main loop for a long time.
  }
  if (Particle.connected()) {
    sysStatus.connectedStatus = true;
    systemStatusWriteNeeded = true;
    return 1;                                                     // Were able to connect successfully
  }
  else {
    return 0;                                                     // Failed to connect
  }
}

bool disconnectFromParticle()                                     // Ensures we disconnect cleanly from Particle
{
  Particle.disconnect();
  waitFor(notConnected, 15000);                                   // make sure before turning off the cellular modem
  Cellular.off();
  sysStatus.connectedStatus = false;
  systemStatusWriteNeeded = true;
  delay(2000);                                                    // Bummer but only should happen once an hour
  return true;
}

bool notConnected() {                                             // Companion function for disconnectFromParticle
  return !Particle.connected();
}

