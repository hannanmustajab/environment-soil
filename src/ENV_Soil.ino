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


/* Alert Code Definitions
* 0 = Normal Operations - No Alert
// device alerts
* 10 = Battery temp too high / low to charge
* 11 = PMIC Reset required
* 12 = Initialization error (likely FRAM)
* 13 = Excessive resets
* 14 = Out of memory
// deviceOS or Firmware alerts
* 20 = Firmware update completed
* 21 = Firmware update timed out
* 22 = Firmware update failed
* 23 = Update attempt limit reached - done for the day
// Connectivity alerts
* 30 = Particle connection timed out but Cellular connection completed
* 31 = Failed to connect to Particle or cellular
// Particle cloud alerts
* 40 = Failed to get Webhook response when connected
*/




// v1.00- ENV Code as starting point. 
// v1.01 - Added soil moisture sensor
// v1.20 - Added second soil moisture sensor to the board. 
// v1.21 - Fixed issue with the size of the data payload. 
// v2.00 - Added publishQueuePosix, new connecting state and fixed low battery issue.



// Particle Product definitions
// PRODUCT_ID(PLATFORM_ID);                            // No longer need to specify - but device needs to be added to product ahead of time.
// PRODUCT_VERSION(1);


#define SOFTWARERELEASENUMBER "2.20"                // Keep track of release numbers

// Included Libraries
#include "math.h"
#include "adafruit-sht31.h"
#include "Adafruit_VEML7700.h"
#include "BackgroundPublishRK.h"
#include "PublishQueuePosixRK.h"                     // Async Particle Publish
#include "MB85RC256V-FRAM-RK.h"                      // Rickkas Particle based FRAM Library
#include "MCP79410RK.h"   


// Define the memory map - note can be EEPROM or FRAM - moving to FRAM for speed and to avoid memory wear
namespace FRAM {                                                                         // Moved to namespace instead of #define to limit scope
  enum Addresses {
    versionAddr           = 0x00,                                                           // Where we store the memory map version number - 8 Bits
    sysStatusAddr         = 0x01,                                                           // This is the status of the device
    sensorDataAddr        = 0xA0                                                            // Where we store the latest sensor data readings
   };
};

const int FRAMversionNumber = 6;                                                            // Increment this number each time the memory map is changed

struct systemStatus_structure {                     
  uint8_t structuresVersion;                        // Version of the data structures (system and data)
  bool thirdPartySim;                               // If this is set to "true" then the keep alive code will be executed
  int keepAlive;                                    // Keep alive value for use with 3rd part SIMs
  bool connectedStatus;
  bool verboseMode;
  bool lowBatteryMode;
  bool lowPowerMode;
  int stateOfCharge;                                // Battery charge level
  unsigned long lastHookResponse;                   // Last time we got a valid Webhook response
  uint8_t placeholdervalue;                         // For future use
  bool verboseCounts;                               // Tells us if we are sending verbose count webhooks
  bool clockSet;                                    // Do we need to do a SyncTime
  bool solarPowerMode;                              // Powered by a solar panel or utility power
  double batteryVoltage;                            // Battery voltage level
  uint8_t batteryState;                             // Stores the current battery state
  int resetCount;                                   // reset count of device (0-256)
  unsigned long lastConnection;                     // Last time we successfully connected to Particle
  uint16_t lastConnectionDuration;                  // How long - in seconds - did it take to last connect to the Particle cloud
} sysStatus;



// Keypad struct for mapping buttons, notes, note values, LED array index, and default color
struct sensor_data_struct {                        // Here we define the structure for collecting and storing data from the sensors
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
  int alerts;                                       // What is the current alert value - see secret decoder ring at top of comments
  uint16_t maxConnectTime = 0;                      // Longest connect time for the hour
  int minBatteryLevel = 100;                        // Lowest Battery level for the day - not sure this is needed
  uint8_t updateAttempts = 0;                       // Number of attempted updates each day

} sensor_data;


// This is the maximum amount of time to allow for connecting to cloud. If this time is
// exceeded, do a deep power down. This should not be less than 10 minutes. 11 minutes
// is a reasonable value to use.
unsigned long connectMaxTimeSec = 10 * 60;          // Timeout for trying to connect to Particle cloud in seconds - reduced to 10 mins

// If updating, we need to delay sleep in order to give the download time to come through before sleeping
const std::chrono::milliseconds firmwareUpdateMaxTime = 10min; // Set at least 5 minutes


// Pin Constants - Boron Carrier Board v1.2a
const int wakeUpPin = D8;                           // This is the Particle Electron WKP pin
const int donePin = D5;
const int userSwitch =    D4;                       // User switch with a pull-up resistor
const int soilPin =      A0;                        // Soil Sensor
const int soilPin2 =      A1;                       // Second Soil Sensor


// Prototypes and System Mode calls
SYSTEM_MODE(SEMI_AUTOMATIC);                        // This will enable user code to start executing automatically.
SYSTEM_THREAD(ENABLED);                                                                     // Means my code will not be held up by Particle processes.
STARTUP(System.enableFeature(FEATURE_RESET_INFO));
SystemSleepConfiguration config;                    // Initialize new Sleep 2.0 Api

Adafruit_VEML7700 veml;
Adafruit_SHT31 sht31 = Adafruit_SHT31();
MB85RC64 fram(Wire, 0);                                                                     // Rickkas' FRAM library
MCP79410 rtc;                                                                               // Rickkas MCP79410 libarary
Timer keepAliveTimer(1000, keepAliveMessage);
FuelGauge fuelGauge;                                // Needed to address issue with updates in low battery state

// State Machine Variables
enum State { INITIALIZATION_STATE, ERROR_STATE, IDLE_STATE, SLEEPING_STATE, NAPPING_STATE, CONNECTING_STATE, REPORTING_STATE, RESP_WAIT_STATE, FIRMWARE_UPDATE};
char stateNames[9][16] = {"Initialize", "Error", "Idle", "Sleeping", "Napping", "Connecting", "Reporting", "Response Wait", "Firmware Update"};
State state = INITIALIZATION_STATE;
State oldState = INITIALIZATION_STATE;

volatile bool watchdogFlag=false;                                                           // Flag to let us know we need to pet the dog


// Timing Variables
int wakeBoundary = 0*3600 + 10*60 + 0;               // Sets a reporting frequency of 1 hour 0 minutes 0 seconds
const unsigned long stayAwakeLong = 90000;          // In lowPowerMode, how long to stay awake every hour
const unsigned long webhookWait = 30000;            // How long will we wait for a WebHook response
const unsigned long resetWait = 30000;              // How long will we wait in ERROR_STATE until reset
unsigned long stayAwakeTimeStamp = 0;               // Timestamps for our timing variables..
unsigned long stayAwake;                            // Stores the time we need to wait before napping
unsigned long resetTimeStamp = 0;                   // Resets - this keeps you from falling into a reset loop
unsigned long lastReportedTime = 0;                 // Need to keep this separate from time so we know when to report
unsigned long connectionStartTime;                  // Timestamp to keep track of how long it takes to connect

// Program Variables
bool dataInFlight = false;                          // Tracks if we have sent data but not yet received a response
bool firmwareUpdateInProgress = false;              // Helps us track if a firmware update is in progress
char SignalString[64];                              // Used to communicate Wireless RSSI and Description
char lowPowerModeStr[16];                           // In low power mode?
bool systemStatusWriteNeeded = false;               // Keep track of when we need to write - system object
bool sensorDataWriteNeeded = false; 


// Variables Related To Particle Mobile Application Reporting
// Simplifies reading values in the Particle Mobile Application
char temperatureString[16];
char humidityString[16];
char batteryContextStr[16];                        // One word that describes whether the device is getting power, charging, discharging or too cold to charge
char batteryString[16];
char luxString[16];                               // String to show the current LUX readings.                         
char whiteString[16];                                                                         // String to show the current threshold readings.                         
char ALSString[16];                                                                         // String to show the current threshold readings.                         

const char* releaseNumber = SOFTWARERELEASENUMBER;                                          // Displays the release on the menu


// This section is where we will initialize sensor specific variables, libraries and function prototypes
// Interrupt Variables
volatile bool userSwitchDetect = false;             // Flag for a user switch press while in connected state


// System Health Variables
int outOfMemory = -1;                               // From reference code provided in AN0023 (see above)


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
  System.on(firmware_update, firmwareUpdateHandler);// Registers a handler that will track if we are getting an update
  System.on(out_of_memory, outOfMemoryHandler);     // Enabling an out of memory handler is a good safety tip. If we run out of memory a System.reset() is done.


  Particle.variable("Release",releaseNumber);
  Particle.variable("temperature", temperatureString);
  Particle.variable("humidity", humidityString);
  Particle.variable("Lux", luxString);
  Particle.variable("White", whiteString);
  Particle.variable("Raw ALS", ALSString);
  Particle.variable("Battery", batteryString);                         // Battery level in V as the Argon does not have a fuel cell
  Particle.variable("BatteryContext",batteryContextStr);
  Particle.variable("Keep Alive Sec",sysStatus.keepAlive);
  Particle.variable("3rd Party Sim", sysStatus.thirdPartySim);
  Particle.variable("lowPowerMode",lowPowerModeStr);
  Particle.variable("Signal", SignalString);
  Particle.variable("ResetCount", sysStatus.resetCount);
  Particle.variable("BatteryVoltage", sysStatus.batteryVoltage);
  Particle.variable("lowPowerMode",lowPowerModeStr);
  Particle.variable("Alerts",sensor_data.alerts);
  Particle.variable("Reporting Duration",String(wakeBoundary));  
  
  Particle.function("Measure-Now",measureNow);
  Particle.function("Verbose-Mode",setVerboseMode);
  Particle.function("Keep Alive",setKeepAlive);
  Particle.function("3rd Party Sim", setThirdPartySim);
  Particle.function("Set Low Power",setLowPowerMode);

  // Particle and System Set up next
  Particle.setDisconnectOptions(CloudDisconnectOptions().graceful(true).timeout(5s));  // Don't disconnect abruptly
  

  // Watchdog Timer and Real Time Clock Initialization
  rtc.setup();                                                                                       // Start the real time clock
  rtc.clearAlarm();                                                                                   // Ensures alarm is still not set from last cycle

  // Take a look at the battery state of charge - good to do this before turning on the cellular modem
  fuelGauge.wakeup();                                                  // Expliciely wake the Feul gauge and give it a half-sec
  delay(500);
  fuelGauge.quickStart();                                              // May help us re-establish a baseline for SoC


  // Next we will load FRAM and check or reset variables to their correct values
  fram.begin();                                                        // Initialize the FRAM module
  byte tempVersion;
  fram.get(FRAM::versionAddr, tempVersion);                            // Load the FRAM memory map version into a variable for comparison
  if (tempVersion != FRAMversionNumber) {                              // Check to see if the memory map in the sketch matches the data on the chip
    fram.erase();                                                      // Reset the FRAM to correct the issue
    fram.put(FRAM::versionAddr, FRAMversionNumber);                    // Put the right value in
    fram.get(FRAM::versionAddr, tempVersion);                          // See if this worked
    if (tempVersion != FRAMversionNumber) {
      state = ERROR_STATE;                                             // Device will not work without FRAM will need to reset
      resetTimeStamp = millis();                                       // Likely close to zero but, for form's sake
      sensor_data.alerts = 12;                                         // FRAM is messed up so can't store but will be read in ERROR state
    }
    else loadSystemDefaults();                                         // Out of the box, we need the device to be awake and connected
  }
  else {
    fram.get(FRAM::sysStatusAddr,sysStatus);                           // Loads the System Status array from FRAM
    fram.get(FRAM::sensorDataAddr,sensor_data);                        // Loead the current values array from FRAM
  }

  if (! sht31.begin(0x44)) {                                                                      // Start the SHT Sensor
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


  checkSystemValues();                                                                      // Make sure System values are all in valid range

   // Take note if we are restarting due to a pin reset - either by the user or the watchdog - could be sign of trouble
  if (System.resetReason() == RESET_REASON_PIN_RESET || System.resetReason() == RESET_REASON_USER) { // Check to see if we are starting from a pin reset or a reset in the sketch
    sysStatus.resetCount++;
    if (sysStatus.resetCount > 3) sensor_data.alerts = 13;               // Excessive resets
  }


  // Publish Queue Posix is used exclusively for sending webhooks and update alerts in order to conserve RAM and reduce writes / wear
  PublishQueuePosix::instance().setup();                             // Start the Publish Queie

  // Next - check to make sure we are not in an endless update loop
  if (sensor_data.updateAttempts >= 3 && sensor_data.alerts != 23) {         // Send out alert the first time we are over the limit
    char data[64];
    System.disableUpdates();                                         // We will only try to update three times in a day
    sensor_data.alerts = 23;                                             // Set an alert that we have maxed out our updates for the day
    snprintf(data, sizeof(data), "{\"alerts\":%i,\"timestamp\":%lu000 }",sensor_data.alerts, Time.now());
    PublishQueuePosix::instance().publish("Ubidots_Alert_Hook", data, PRIVATE); // Put in publish queue
  }

  (sysStatus.lowPowerMode) ? strncpy(lowPowerModeStr,"True",sizeof(lowPowerModeStr)) : strncpy(lowPowerModeStr,"False",sizeof(lowPowerModeStr));


  if (sysStatus.thirdPartySim) {
    waitUntil(Particle.connected); 
    Particle.keepAlive(sysStatus.keepAlive);                                                    // Set the keep alive value
    keepAliveTimer.changePeriod(sysStatus.keepAlive*1000);                                  // Will start the repeating timer
  }
  
  if (!digitalRead(userSwitch)) loadSystemDefaults();                                       // Make sure the device wakes up and connects
  
  takeMeasurements();                                                                       // For the benefit of monitoring the device
  
  setLowPowerMode("0");

  // if (sysStatus.lowBatteryMode) setLowPowerMode("1");                                       // If battery is low we need to go to low power state

  if(sysStatus.verboseMode) PublishQueuePosix::instance().publish("Startup",StartupMessage,PRIVATE);                       // Let Particle know how the startup process went

  if (state == INITIALIZATION_STATE) state = CONNECTING_STATE;                                    // We made it throughgo let's go to idle
}

void loop()
{

  switch(state) {
  
  case IDLE_STATE:                                                     // Where we spend most time - note, the order of these conditionals is important
    if (state != oldState) publishStateTransition();
    if (sysStatus.lowPowerMode && (millis() - stayAwakeTimeStamp) > stayAwake) state = NAPPING_STATE;     
    if (firmwareUpdateInProgress) state= FIRMWARE_UPDATE;                                                     // This means there is a firemware update on deck
    // if (Time.hour() != Time.hour(lastReportedTime)) state = REPORTING_STATE;                                  // We want to report on the hour but not after bedtime
    if (!(Time.now() % wakeBoundary)) state = REPORTING_STATE;  
    break;

   case REPORTING_STATE:                                                // In this state we will publish the hourly totals to the queue and decide whether we should connect
    if (state != oldState) publishStateTransition();
    lastReportedTime = Time.now();                                     // We are only going to report once each hour from the IDLE state.  We may or may not connect to Particle
    takeMeasurements();                                                // Take Measurements here for reporting
    sendEvent();                                                       // Publish hourly but not at opening time as there is nothing to publish
    state = CONNECTING_STATE;                                          // Default behaviour would be to connect and send report to Ubidots

    // Let's see if we need to connect 
    if (Particle.connected()) {                                        // We are already connected go to response wait
      stayAwake = stayAwakeLong;                                       // Keeps device awake after reboot - helps with recovery
      stayAwakeTimeStamp = millis();
      state = RESP_WAIT_STATE;
    }
    // If we are in a low battery state - we are not going to connect unless we are over-riding with user switch (active low)
    else if (sysStatus.lowBatteryMode && digitalRead(userSwitch)) {
      Log.info("Not connecting - low battery mode");
      state = IDLE_STATE;
    }
    // If we are in low power mode, we may bail if battery is too low and we need to reduce reporting frequency
    else if (sysStatus.lowPowerMode && digitalRead(userSwitch)) {      // Low power mode and user switch not pressed
      if (sysStatus.stateOfCharge > 65) {
        Log.info("Sufficient battery power connecting");
      }
      else if (sysStatus.stateOfCharge <= 50 && (Time.hour() % 4)) {   // If the battery level is <50%, only connect every fourth hour
        Log.info("Not connecting - <50%% charge - four hour schedule");
        state = IDLE_STATE;                                            // Will send us to connecting state - and it will send us back here
      }                                                                // Leave this state and go connect - will return only if we are successful in connecting
      else if (sysStatus.stateOfCharge <= 65 && (Time.hour() % 2)) {   // If the battery level is 50% -  65%, only connect every other hour
        Log.info("Not connecting - 50-65%% charge - two hour schedule");
        state = IDLE_STATE;                                            // Will send us to connecting state - and it will send us back here
        break;                                                         // Leave this state and go connect - will return only if we are successful in connecting
      }
    }
    break;

   case RESP_WAIT_STATE: {
    static unsigned long webhookTimeStamp = 0;                        // Webhook time stamp
    if (state != oldState) {
      webhookTimeStamp = millis();                                    // We are connected and we have published, head to the response wait state
      dataInFlight = true;                                            // set the data inflight flag
      publishStateTransition();
    }
    if (!dataInFlight)  {                                             // Response received --> back to IDLE state
      state = IDLE_STATE;
    }
    else if (millis() - webhookTimeStamp > webhookWait) {             // If it takes too long - will need to reset
      sensor_data.alerts = 40;                                            // Raise the missed webhook flag
      sensorDataWriteNeeded = true;
      if (Time.now() - sysStatus.lastHookResponse > 3 * 3600L) {      // Failed to get a webhook response for over three hours
        resetTimeStamp = millis();
        state = ERROR_STATE;                                          // Response timed out
      }
      else state = IDLE_STATE;
    }
    } break;

    case NAPPING_STATE: {                                                // This state puts the device in low power mode quickly - napping supports the sensor activity and interrupts
      if (state != oldState) publishStateTransition();
      if (Particle.connected() || !Cellular.isOff()) disconnectFromParticle();           // Disconnect cleanly from Particle and power down the modem
      stayAwake = 1000;                                                  // Once we come into this function, we need to reset stayAwake as it changes at the top of the hour
      // state = IDLE_STATE;                                                // Back to the IDLE_STATE after a nap - not enabling updates here as napping is typicallly disconnected
      int wakeInSeconds = constrain(wakeBoundary - Time.now() % wakeBoundary, 1, wakeBoundary);
      Log.info("Napping for %i seconds",wakeInSeconds);
      PublishQueuePosix::instance().publish("Napping Duration", String(wakeInSeconds), PRIVATE);      
      config.mode(SystemSleepMode::ULTRA_LOW_POWER)
        .gpio(userSwitch,CHANGE)                                         // User presses the user button
        .duration(wakeInSeconds * 1000);
      SystemSleepResult result = System.sleep(config);                   // Put the device to sleep
      fuelGauge.wakeup();                                                // Make sure the fuelGauge is woke
      stayAwakeTimeStamp = millis();
      if (result.wakeupPin() == userSwitch) setLowPowerMode("0");        // The user woke the device and we need to make sure it stays awake
      state = REPORTING_STATE;
    } break;

    case CONNECTING_STATE:{                                              // Will connect - or not and head back to the Idle state
      static State retainedOldState;                                     // Keep track for where to go next (depends on whether we were called from Reporting)
      static unsigned long connectionStartTimeStamp;                     // Time in Millis that helps us know how long it took to connect

      if (state != oldState) {                                           // Non-blocking function - these are first time items
        retainedOldState = oldState;                                     // Keep track for where to go next
        sysStatus.lastConnectionDuration = 0;                            // Will exit with 0 if we do not connect or are connected or the connection time if we do
        publishStateTransition();

        // Let's make sure we need to connect
        if (Particle.connected()) {
          Log.info("Connecting state but already connected");
          stayAwake = stayAwakeLong;                                     // Keeps device awake after reboot - helps with recovery
          stayAwakeTimeStamp = millis();
          (retainedOldState == REPORTING_STATE) ? state = RESP_WAIT_STATE : state = IDLE_STATE;
          break;
        }

        // OK, let's do this thing!
        connectionStartTimeStamp = millis();                             // Have to use millis as the clock will get reset on connect
        Cellular.on();                                                   // Needed until they fix this: https://github.com/particle-iot/device-os/issues/1631
        Particle.connect();                                              // Told the Particle to connect, now we need to wait
      }

      sysStatus.lastConnectionDuration = int((millis() - connectionStartTimeStamp)/1000);

      if (Particle.connected()) {
        Particle.syncTime();                                           // Set the clock each day
        waitFor(Particle.syncTimeDone,30000);                          // Wait for up to 30 seconds for the SyncTime to complete
        
        sysStatus.lastConnection = Time.now();                           // This is the last time we attempted to connect
        stayAwake = stayAwakeLong;                                       // Keeps device awake after reboot - helps with recovery
        stayAwakeTimeStamp = millis();
        recordConnectionDetails();                                       // Record outcome of connection attempt
        attachInterrupt(userSwitch, userSwitchISR,FALLING);              // Attach interrupt for the user switch to enable verbose counts
        (retainedOldState == REPORTING_STATE) ? state = RESP_WAIT_STATE : state = IDLE_STATE;
      }
      else if (sysStatus.lastConnectionDuration > connectMaxTimeSec) {
        recordConnectionDetails();                                       // Record outcome of connection attempt
        Log.info("cloud connection unsuccessful");
        disconnectFromParticle();                                        // Make sure the modem is turned off
        if (sysStatus.solarPowerMode) setLowPowerMode("1");              // If we cannot connect, there is no point to stayng out of low power mode
        if ((Time.now() - sysStatus.lastConnection) > 6 * 3600L) {       // Only sends to ERROR_STATE if it has been over six hours - this ties to reporting and low battery state
          state = ERROR_STATE;
          resetTimeStamp = millis();
          break;
        }
        else state = IDLE_STATE;
      }
    }break;

     case ERROR_STATE: {                                                  // New and improved - now looks at the alert codes
    if (state != oldState) publishStateTransition();                   // We will apply the back-offs before sending to ERROR state - so if we are here we will take action
    if (millis() > resetTimeStamp + resetWait) {                       // This simply gives us some time to catch the device if it is in a reset loop                           
      char errorStr[64];                                             // Let's publish to let folks know what is going on
      snprintf(errorStr, sizeof(errorStr),"Resetting device with alert code %i",sensor_data.alerts);
      if (Particle.connected()) Particle.publish("ERROR_STATE", errorStr, PRIVATE);
      Log.info(errorStr);
      delay(2000);

      if (Particle.connected() || !Cellular.isOff()) disconnectFromParticle();  // Disconnect cleanly from Particle and power down the modem

      switch (sensor_data.alerts) {                                        // All of these events will reset the device
        case 12:                                                       // This is an initialization error - likely FRAM - need to power cycle to clear
          System.reset();                                    // 30 second power cycle of Boron including cellular modem, carrier board and all peripherals
          break;

        case 30 ... 31:                                                // Device failed to connect too many times
          sysStatus.lastConnection = Time.now();                       // Make sure we don't do this very often
          fram.put(FRAM::sysStatusAddr,sysStatus);                  // Unless a FRAM error sent us here - store alerts value
          delay(100);                                                  // Time to write to FRAM
          System.reset();
          break;

        case 13:                                                       // Excessive resets of the device - time to power cycle
          sysStatus.resetCount = 0;                                    // Reset so we don't do this too often
          fram.put(FRAM::sysStatusAddr,sysStatus);                  // Won't get back to the main loop
          delay (100);
          // ab1805.deepPowerDown();                                      // 30 second power cycle of Boron including cellular modem, carrier board and all peripherals
          break;

        case 14:                                                       // This is an out of memory error
          System.reset();
          break;

        case 40:                                                       // This is for failed webhook responses over three hours
          System.reset();
          break;

        default:                                                        // Make sure that, no matter what - we do not get stuck here
          System.reset();
          break;
      }
    }
    } break;

  case FIRMWARE_UPDATE: {
      static unsigned long stateTime;
      char data[64];

      if (state != oldState) {
        stateTime = millis();                                          // When did we start the firmware update?
        Log.info("In the firmware update state");
        publishStateTransition();
      }
      if (!firmwareUpdateInProgress) {                                 // Done with the update
          Log.info("firmware update completed");
          state = IDLE_STATE;
      }
      else
      if (millis() - stateTime >= firmwareUpdateMaxTime.count()) {     // Ran out of time
          Log.info("firmware update timed out");
          sensor_data.alerts = 21;                                          // Record alert for timeout
          snprintf(data, sizeof(data), "{\"alerts\":%i,\"timestamp\":%lu000 }",sensor_data.alerts, Time.now());
          PublishQueuePosix::instance().publish("Ubidots_Alert_Hook", data, PRIVATE);
          sensor_data.updateAttempts++;                                    // Increment the update attempt counter
          state = IDLE_STATE;
      }
    } break;

  }

  // Take care of housekeeping items here
  
  PublishQueuePosix::instance().loop();                                // Check to see if we need to tend to the message queue
  
  rtc.loop();                   // keeps the clock up to date

   if (outOfMemory >= 0) {                                              // In this function we are going to reset the system if there is an out of memory error
    sensor_data.alerts = 14;                                               // Out of memory alert
    resetTimeStamp = millis();
    state = ERROR_STATE;
  }

  if (watchdogFlag) petWatchdog();                                     // Watchdog flag is raised - time to pet the watchdog

  if (systemStatusWriteNeeded) {
    fram.put(FRAM::sysStatusAddr,sysStatus);
    systemStatusWriteNeeded = false;
  }

  if (sensorDataWriteNeeded) {
    fram.put(FRAM::sensorDataAddr,sensor_data);
    sensorDataWriteNeeded = false;
  }

    // End of housekeeping - end of main loop

}


void loadSystemDefaults() {                                           // Default settings for the device - connected, not-low power and always on
  if (Particle.connected()) {
    waitUntil(meterParticlePublish);
    Particle.publish("Mode","Loading System Defaults", PRIVATE);
  }
  Log.info("Loading system defaults");
  sysStatus.structuresVersion = 1;
  sysStatus.verboseMode = true;
  sysStatus.lowBatteryMode = false;
  if (digitalRead(userSwitch)) setLowPowerMode("1");                  // Low power mode or not depending on user switch
  else setLowPowerMode("0");

  sysStatus.solarPowerMode = true;
  sysStatus.lastConnectionDuration = 0;                               // New measure
  fram.put(FRAM::sysStatusAddr,sysStatus);                         // Write it now since this is a big deal and I don't want values over written
}


 /**
  * @brief This function checks to make sure all values that we pull from FRAM are in bounds
  *
  * @details As new devices are comissioned or the sysStatus structure is changed, we need to make sure that values are
  * in bounds so they do not cause unpredectable execution.
  *
  */
void checkSystemValues() {                                          // Checks to ensure that all system values are in reasonable range
  if (sysStatus.resetCount < 0 || sysStatus.resetCount > 255) sysStatus.resetCount = 0;
  if (sysStatus.lastConnectionDuration > connectMaxTimeSec) sysStatus.lastConnectionDuration = 0;
  if (sensor_data.maxConnectTime > connectMaxTimeSec) {
    sensor_data.maxConnectTime = 0;
    sensorDataWriteNeeded = true;
  }

  // None for lastHookResponse
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
  PublishQueuePosix::instance().publish("environmental-hook", data, PRIVATE | WITH_ACK);
  // dataInFlight = true;                                                                      // set the data inflight flag
  // webhookTimeStamp = millis();
  sensor_data.alerts = 0;                                                 // Reset the alert after publish
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
  if (sysStatus.verboseMode && Particle.connected()) {
    waitUntil(meterParticlePublish);
    Particle.publish("Ubidots Hook", responseString, PRIVATE);
  }
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
    snprintf(batteryString, sizeof(batteryString), "%i ", sensor_data.stateOfCharge);
   
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
    state = REPORTING_STATE;
    return 1;
  }
  else return 0;
}

int setVerboseMode(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    sysStatus.verboseMode = true;
    PublishQueuePosix::instance().publish("Mode","Set Verbose Mode",PRIVATE);
    systemStatusWriteNeeded = true;
    return 1;
  }
  else if (command == "0")
  {
    sysStatus.verboseMode = false;
    PublishQueuePosix::instance().publish("Mode","Cleared Verbose Mode",PRIVATE);
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
  if(Particle.connected()) PublishQueuePosix::instance().publish("State Transition",stateTransitionString, PRIVATE);
}



int setThirdPartySim(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    sysStatus.thirdPartySim = true;
    Particle.keepAlive(sysStatus.keepAlive);                                                // Set the keep alive value
    keepAliveTimer.changePeriod(sysStatus.keepAlive*1000);                                  // Will start the repeating timer
    if (Particle.connected()) PublishQueuePosix::instance().publish("Mode","Set to 3rd Party Sim", PRIVATE);
    systemStatusWriteNeeded = true;
    return 1;
  }
  else if (command == "0")
  {
    sysStatus.thirdPartySim = false;
    if (Particle.connected()) PublishQueuePosix::instance().publish("Mode","Set to Particle Sim", PRIVATE);
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
    PublishQueuePosix::instance().publish("Keep Alive",data, PRIVATE);
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
    sysStatus.lowPowerMode = true;
    Log.info("Set Low Power Mode");
    if (Particle.connected()) {
      waitUntil(meterParticlePublish);
      Particle.publish("Mode",lowPowerModeStr, PRIVATE);
    }
  }
  else if (command == "0")                                            // Command calls for clearing lowPowerMode
  {
    sysStatus.lowPowerMode = false;
    Log.info("Cleared Low Power Mode");
    if (!Particle.connected()) {                                 // In case we are not connected, we will do so now.
      state = CONNECTING_STATE;                                       // Will connect - if connection fails, will need to reset device
    }
    else {
      waitUntil(meterParticlePublish);
      Particle.publish("Mode",lowPowerModeStr, PRIVATE);
    }
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
  PublishQueuePosix::instance().publish("*", PRIVATE,NO_ACK);
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



void getSignalStrength() {
  const char* radioTech[10] = {"Unknown","None","WiFi","GSM","UMTS","CDMA","LTE","IEEE802154","LTE_CAT_M1","LTE_CAT_NB1"};
  // New Signal Strength capability - https://community.particle.io/t/boron-lte-and-cellular-rssi-funny-values/45299/8
  CellularSignal sig = Cellular.RSSI();

  auto rat = sig.getAccessTechnology();

  //float strengthVal = sig.getStrengthValue();
  float strengthPercentage = sig.getStrength();

  //float qualityVal = sig.getQualityValue();
  float qualityPercentage = sig.getQuality();

  snprintf(SignalString,sizeof(SignalString), "%s S:%2.0f%%, Q:%2.0f%% ", radioTech[rat], strengthPercentage, qualityPercentage);
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



void firmwareUpdateHandler(system_event_t event, u_int param) {
  switch(param) {
    char data[64];                                                     // Store the date in this character array - not global

    case firmware_update_begin:
      firmwareUpdateInProgress = true;
      break;
    case firmware_update_complete:
      firmwareUpdateInProgress = false;
      sensor_data.alerts = 20;                                             // Record a successful attempt
      snprintf(data, sizeof(data), "{\"alerts\":%i,\"timestamp\":%lu000 }",sensor_data.alerts, Time.now());
      PublishQueuePosix::instance().publish("Ubidots_Alert_Hook", data, PRIVATE); // Put in publish queue
      sensor_data.updateAttempts = 0;                                      // Zero the update attempts counter
      break;
    case firmware_update_failed:
    firmwareUpdateInProgress = false;
      sensor_data.alerts = 22;                                             // Record a failed attempt
      snprintf(data, sizeof(data), "{\"alerts\":%i,\"timestamp\":%lu000 }",sensor_data.alerts, Time.now());
      PublishQueuePosix::instance().publish("Ubidots_Alert_Hook", data, PRIVATE); // Put in publlish queue
      sensor_data.updateAttempts++;                                        // Increment the update attempts counter
      break;
  }
  sensorDataWriteNeeded = true;
}

// Here are the various hardware and timer interrupt service routines
void outOfMemoryHandler(system_event_t event, int param) {
    outOfMemory = param;
}


void  recordConnectionDetails()  {                                     // Whether the connection was successful or not, we will collect and publish metrics
  char data[64];

  if (sysStatus.lastConnectionDuration > connectMaxTimeSec+1) sysStatus.lastConnectionDuration = 0;
  else if (sysStatus.lastConnectionDuration > sensor_data.maxConnectTime) sensor_data.maxConnectTime = sysStatus.lastConnectionDuration; // Keep track of longest each day

  if (Cellular.ready()) getSignalStrength();                           // Test signal strength if the cellular modem is on and ready

  snprintf(data, sizeof(data),"Connected in %i secs",sysStatus.lastConnectionDuration);                   // Make up connection string and publish
  Log.info(data);

  if (Particle.connected()) {
    Log.info("Cloud connection successful");
    if (sysStatus.verboseMode) Particle.publish("Cellular",data,PRIVATE);
  }
  else if (Cellular.ready()) {                                        // We want to take note of this as it implies an issue with the Particle back-end
    Log.info("Connected to cellular but not Particle");
    sensor_data.alerts = 30;                                              // Record alert for timeout on Particle but connected to cellular
    snprintf(data, sizeof(data), "{\"alerts\":%i,\"timestamp\":%lu000 }",sensor_data.alerts, Time.now());
    PublishQueuePosix::instance().publish("Ubidots_Alert_Hook", data, PRIVATE);
  }
  else {
    Log.info("Failed to connect");
    sensor_data.alerts = 31;                                              // Record alert for timeout on cellular
    snprintf(data, sizeof(data), "{\"alerts\":%i,\"timestamp\":%lu000 }",sensor_data.alerts, Time.now());
    PublishQueuePosix::instance().publish("Ubidots_Alert_Hook", data, PRIVATE);
  }

  systemStatusWriteNeeded = true;
  sensorDataWriteNeeded = true;
}


void userSwitchISR() {
  userSwitchDetect = true;                                            // The the flag for the user switch interrupt
}


bool meterParticlePublish() {
  static unsigned long lastPublish = 0;  
  
  if (millis() - lastPublish >= 1000) return 1;
  
  return 0;
}