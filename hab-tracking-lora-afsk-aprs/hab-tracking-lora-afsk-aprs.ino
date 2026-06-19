#include <TinyGPSPlus.h>
#include <MemoryFree.h>
#include <Wire.h>
#include <RadioLib.h>
#include <avr/dtostrf.h>
#include "SparkFunBME280.h"
#include <ZeroAPRS.h>
#include <ZeroSi4463.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "LightHABLib.h"

//LoRa Pin Definitions
#define LORA_NSS_PIN  0
#define LORA_RST_PIN  13
#define LORA_DIO1_PIN 12
#define LORA_BUSY_PIN 10

//AFSK Pin Definitions
#define SI446X_NIRQ_PIN   3
#define SI446X_SDN_PIN    11
#define SI446X_NSEL_PIN   1
#define SI4463_ON    digitalWrite(SI446X_SDN_PIN, LOW)
#define SI4463_OFF   digitalWrite(SI446X_SDN_PIN, HIGH)

//GPS Pin Definitions
#define GPS_RESET_PIN    32
#define GPS_FORCE_ON_PIN 7
#define GPS_JAM_DET_PIN  A3
#define GPS_LED_GND_PIN  9

//Pyro Pin Definitions
#define PYRO_PIN_1  4
#define PYRO_PIN_2  5

#define PYRO1_ON  digitalWrite(PYRO_PIN_1, HIGH);
#define PYRO1_OFF digitalWrite(PYRO_PIN_1, LOW);

#define PYRO2_ON  digitalWrite(PYRO_PIN_2, HIGH);
#define PYRO2_OFF digitalWrite(PYRO_PIN_2, LOW);

//Status LED
#define STATUS_LED_PIN  8
#define STATUS_LED_ON  digitalWrite(STATUS_LED_PIN, HIGH)
#define STATUS_LED_OFF digitalWrite(STATUS_LED_PIN, LOW)

//Flash LED
#define FLASH_LED_PIN A4
#define FLASH_LED_ON  digitalWrite(FLASH_LED_PIN, LOW)
#define FLASH_LED_OFF digitalWrite(FLASH_LED_PIN, HIGH)

//Buzzer
#define BUZZER_PIN  6
#define BUZZER_ON  digitalWrite(BUZZER_PIN, HIGH)
#define BUZZER_OFF digitalWrite(BUZZER_PIN, LOW)

//Battery
#define BATT_PIN A5

//Cutdown
#define CUTDOWN_PIN PYRO_PIN_1
#define CUTDOWN_ON  digitalWrite(CUTDOWN_PIN, HIGH);
#define CUTDOWN_OFF digitalWrite(CUTDOWN_PIN, LOW);

TinyGPSPlus gps;
BME280 myBME;

Module radioModule(uint32_t(LORA_NSS_PIN), LORA_DIO1_PIN, LORA_RST_PIN, LORA_BUSY_PIN);
SX1268 radio(&radioModule);
Si4463 si4463(SI446X_NIRQ_PIN, SI446X_SDN_PIN, SI446X_NSEL_PIN);

//#define DEVMODE // Development mode. Uncomment to enable for debugging.

//******************************  AFSK APRS (VHF) SETTINGS **********************************

struct AFSKConfig {
  bool enabled;
  char callSign[7];
  int8_t callNumber;
  char symbol;
  bool alternateSymbolTable;
  char statusMessage[51];   //Max 50 characters.
  char comment[51];         //Max 50 characters.
  uint32_t frequency;
  int32_t freqCorrection;
  uint32_t flightInterval;
  uint32_t groundInterval;
  uint8_t wide1;
  uint8_t wide2;
  uint8_t pathSize;
  bool sendEnhancedPrecision;
};

AFSKConfig afskConfig = {
  false,             // enabled
  "N0CALL",         // callSign - DO NOT FORGET TO CHANGE YOUR CALLSIGN
  11,                // callNumber (SSID) http://www.aprs.org/aprs11/SSIDs.txt
  '>',              // symbol - 'O' for balloon, '>' for car, for more info : http://www.aprs.org/symbols/symbols-new.txt
  false,            // alternateSymbolTable - false = '/' , true = '\'
  "LightHABTracker by TA2MUN & TA2WX - AFSK", // statusMessage
  "",               // comment
  144800000,        // frequency - AFSK TX frequency 144800000 for Europe
  -1200,            // freqCorrection - Hz. vctcxo frequency correction for si4463
  65000,            // flightInterval - Schedule AFSK beacon every this many miliseconds during flight Default 1 minute
  300000,           // groundInterval - Schedule AFSK beacon every this many miliseconds on the ground Default 5 minute
  1,                // wide1 - 1 for WIDE1-1 path
  1,                // wide2 - 1 for WIDE2-1 path
  2,                // pathSize - 2 for WIDE1-N,WIDE2-N ; 1 for WIDE2-N
  true              // sendEnhancedPrecision
};

//************************** LoRa APRS (UHF) Settings ********************

struct LoRaConfig {
  bool enabled;
  char callSign[12];
  char symbolCode[2];
  char symbolTable[2];
  char statusMessage[51];
  char comment[51];
  char wide[12];
  char header[4];           //Header for https://github.com/lora-aprs/LoRa_APRS_iGate compatibility
  float frequency;
  float bandwidth;
  uint8_t spreadingFactor;
  uint8_t codingRate;
  int8_t outputPower;
  uint16_t preambleLength;
  int8_t crc;
  uint32_t flightInterval;
  uint32_t groundInterval;
};

LoRaConfig loraConfig = {
  true,             // enabled
  "N0CALL-13",      // callSign
  "O",              // symbolCode
  "/",              // symbolTable
  "LightHABTracker by TA2MUN & TA2WX - LoRa", // statusMessage
  "",               // comment
  "WIDE1-1",        // wide
  "<\xff\x01",      // header
  433.775f,         // frequency
  125.0f,           // bandwidth
  12,               // spreadingFactor
  5,                // codingRate
  22,               // outputPower
  8,                // preambleLength
  1,                // crc
  60000,            // flightInterval - Schedule LoRa beacon every this many miliseconds during flight Default 1 minute
  180000            // groundInterval - Schedule LoRa beacon every this many miliseconds on the ground Default 3 minute
};

//************ CUT DOWN Settings ************
struct CutDownConfig {
  bool enabled;
  char secret[16];
};

CutDownConfig cutDownConfig = {
  false,            // enabled
  "ABC123"          // secret
};

bool cutDownInProgress = false;
uint32_t lastCutDown = 0;
uint16_t cutDownCommandCount = 0;

//********************************* Power Settings ******************************

struct PowerConfig {
  int waitTime;
  float minVoltage;
  float lowPowerVoltage;
  uint32_t lowPowerInterval;
};

PowerConfig powerConfig = {
  60000,            // waitTime - Miliseconds sleep if batteries are below minVoltage
  2.7f,             // minVoltage - Minimum volts to operate
  3.5f,             // lowPowerVoltage - Below this, intervals are increased to 30 minutes
  1800000           // lowPowerInterval - Miliseconds (30 minutes)
};

//************************ Ascent/Descent Detection Settings ************************

struct FlightConfig {
  uint16_t altitudeCheckInterval;
  bool useBarometricAltitudeNoGPSFix;
  float deltaAscDescTrigger;
  bool predictLanding;
};

FlightConfig flightConfig = {
  10000,            // altitudeCheckInterval - Miliseconds. Delay time for altitude check
  true,             // useBarometricAltitudeNoGPSFix - Use pressure sensor for altitude if GPS failed
  100.0f,           // deltaAscDescTrigger - Feet. Trigger difference for flight phases change.
  true              // predictLanding
};

#define ALTITUDE_SAMPLES_SHORT 5  // Number of samples for short moving average
#define ALTITUDE_SAMPLES_LONG 30  // Number of samples for long moving average

enum FlightPhases {
  INITIALIZING = -1,
  READY_TO_LAUNCH = 0,
  ASCENDING = 1,
  DESCENDING = 2,
  LANDED = 3,
  RECOVERY = 4,
};

int8_t flightStatus = INITIALIZING; //-1:INITIALIZING 0:READY TO LAUNCH 1:ASCENDING 2:DESCENDING 3:LANDED 4:RECOVERY

//********************************* Flash LED Settings ******************************
bool enableFlashLEDBlinkInLoop = true; //Set false if you don't need Flash LED
bool doNotBlinkDuringDay = true; //Do not blink Flash LED at day light to save the batteries
uint8_t flightStatusLevelForLEDBlink = 1; //3:Only during landed 2: Descending and landed 1: Ascending, descending and landed 0: All the time
float maxFlashLEDAltitude = 10000.f; //Feet. Altitude limit for Flash LED, it won't blink above this limit
uint16_t flashLEDBlinkInterval = 60000; //Miliseconds. Delay time for Flash LED ON
uint8_t flashLEDBlinkCount = 5;
uint16_t flashLEDBlinkTimeON = 100; //Miliseconds. Delay time while LED ON
uint16_t flashLEDBlinkTimeOFF = 300; //Miliseconds. Delay time while LED OFF

//************************ Buzzer Settings ************************
bool enableBuzzerInLoop = true; //Sound buzzer regularly if balloon is landed
uint16_t buzzerSoundInterval = 60000; //Miliseconds. Delay time for buzzer sound
uint8_t soundBuzzerCount = 5;
uint16_t soundBuzzerTimeON = 200; //Miliseconds. Delay time while Buzzer ON
uint16_t soundBuzzerTimeOFF = 300; //Miliseconds. Delay time while Buzzer OFF

//************************ Global State Variables ************************
float altitudeSamplesShortBuf[ALTITUDE_SAMPLES_SHORT];
float altitudeSamplesLongBuf[ALTITUDE_SAMPLES_LONG];
MovingAverage altitudeAvgShort;
MovingAverage altitudeAvgLong;

unsigned long altitudeLastProcessedTimeStamp = 0;
float launchAltitudeAvg = 0;

float batteryVoltage;
uint32_t lastBuzzerSounded = 0;
uint32_t lastFlashLEDBlinked = 0;
unsigned long gpsLastProcessedTimeStamp = 0;
bool gpsFixSearchLed = true;
bool gpsFix = false;
bool gpsStandbyMode = false;

uint16_t txCount = 1;
boolean packetQueued = false;
unsigned long lastLoRaTXPacketTime = -1000000;
unsigned long lastAFSKTXPacketTime = -1000000;

// flag to indicate that a LoRa packet was received
volatile bool receivedFlag = false;

LandingPredictor predictor;

// this function is called when a complete packet
// is received by the module
void setFlag(void) {
  receivedFlag = true;
}

//************************ GPS Data Helpers ************************

double getLatitude() {
  return gps.location.lat();
}

double getLongitude() {
  return gps.location.lng();
}

double getAltitudeMeters() {
  return gps.altitude.meters();
}

double getAltitudeFeet() {
  return gps.altitude.feet();
}

float getSpeedKnots() {
  return (float)gps.speed.knots();
}

float getCourseDeg() {
  return (float)gps.course.deg();
}

uint8_t getSatellites() {
  return (uint8_t)gps.satellites.value();
}

uint8_t getHour() {
  return gps.time.hour();
}

uint8_t getMinute() {
  return gps.time.minute();
}

uint8_t getSecond() {
  return gps.time.second();
}

uint16_t getDayOfYear() {
  return hab_calculateDayOfYear(gps.date.year(), gps.date.month(), gps.date.day());
}

bool hasValidFix() {
  return gpsFix;
}

bool testGPSCommandEcho() {
  const char *cmd = "$PMTK605*31\r\n";

  SerialUSB.println("Sending command to GPS:");
  SerialUSB.println(cmd);

  Serial.print(cmd);

  unsigned long start = millis();
  String input = "";

  while (millis() - start < 2000) {
    while (Serial.available()) {
      char c = Serial.read();
      input += c;
    }
  }

  SerialUSB.println("Received from GPS:");
  SerialUSB.println(input);

  return input.length() > 0;
}

void setup() {
  pinMode(FLASH_LED_PIN, OUTPUT); FLASH_LED_OFF;
  pinMode(GPS_LED_GND_PIN, OUTPUT);
  pinMode(GPS_RESET_PIN, OUTPUT);
  pinMode(GPS_FORCE_ON_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(PYRO_PIN_1, OUTPUT);
  pinMode(PYRO_PIN_2, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);

  delay(100);
  digitalWrite(GPS_LED_GND_PIN, LOW);
  digitalWrite(GPS_RESET_PIN, HIGH);

  Wire.begin();
  SerialUSB.begin(115200);

  // Wait up to 5 seconds for serial to be opened
  unsigned long start = millis();
  while (millis() - start < 5000 && !SerialUSB);
  delay(5000);
  SerialUSB.println();
  SerialUSB.println(F("Starting"));
  SerialUSB.println();
  freeMem();
  printUserInfo();
  setupGPS();
  setupBME280();
  setupLoRa();
  setupAFSK();

  delay(1000);
  if (loraConfig.enabled) {
    sendLoRaStatusMessage();
  }

  if (afskConfig.enabled) {
    sendAFSKStatus();
  }
  delay(5000);

  // Initialize moving averages
  hab_ma_init(&altitudeAvgShort, altitudeSamplesShortBuf, ALTITUDE_SAMPLES_SHORT);
  hab_ma_init(&altitudeAvgLong, altitudeSamplesLongBuf, ALTITUDE_SAMPLES_LONG);

  // Initialize landing predictor (targets in feet)
  uint16_t targets[LP_NUM_TARGETS] = {13000, 10000, 6500, 3300};
  lp_init(&predictor, targets);

  freeMem();
}

void checkBattery() {
  batteryVoltage = readBatt();

  bool readyToTXAFSK = (afskConfig.enabled && millis() - lastAFSKTXPacketTime > powerConfig.lowPowerInterval);
  bool readyToTXLoRa = (loraConfig.enabled && millis() - lastLoRaTXPacketTime > powerConfig.lowPowerInterval);

  if ((readyToTXAFSK || readyToTXLoRa) && gpsStandbyMode) {
    SerialUSB.println("Exiting GPS standby mode");
    setGPSStandbyMode(false);
  } else if (batteryVoltage < powerConfig.lowPowerVoltage && flightStatus == LANDED && !gpsStandbyMode && !(readyToTXAFSK || readyToTXLoRa)) {
    SerialUSB.println("Entering GPS standby mode");
    setGPSStandbyMode(true);
  }
}

void loop() {

  checkBattery();

  if (batteryVoltage > powerConfig.minVoltage) {

    if (cutDownInProgress && (millis() - lastCutDown > 10000)) {
      SerialUSB.println(F("Cut Down Command Completed..."));
      CUTDOWN_OFF;
      cutDownInProgress = false;
    }

    checkGPSStatus();
    checkFlightStatus();
    loopBuzzer();
    loopFlashLED();
    loopLoRaTX();
    loopAFSKTX();
    freeMem();

    if (receivedFlag) {
      receivedFlag = false;
      SerialUSB.println(F("[SX1268] Received packet!"));
      parseRXPacket();
    }

  } else {
    delay(powerConfig.waitTime);
  }
}

void printUserInfo() {
  SerialUSB.print(F("LoRa APRS (UHF) CallSign: "));
  SerialUSB.println(loraConfig.callSign);
  SerialUSB.println(F(""));
  SerialUSB.print(F("AFSK APRS (VHF) CallSign: "));
  SerialUSB.print(afskConfig.callSign);
  SerialUSB.print(F("-"));
  SerialUSB.println(afskConfig.callNumber);
}

void loopLoRaTX() {
  uint32_t beaconLoRaInterval = 0;

  if (flightStatus == ASCENDING || flightStatus == DESCENDING) {
    beaconLoRaInterval = loraConfig.flightInterval;
  } else {
    beaconLoRaInterval = loraConfig.groundInterval;
  }

  if (batteryVoltage < powerConfig.lowPowerVoltage && flightStatus == LANDED) {
    beaconLoRaInterval = powerConfig.lowPowerInterval;
  }

  if (loraConfig.enabled && millis() - lastLoRaTXPacketTime > beaconLoRaInterval) {
    if (gpsFix) {
      sendLoRaLocationMessage(true);
      lastLoRaTXPacketTime = millis();
      freeMem();
    }
  }
}

void loopAFSKTX() {
  uint32_t beaconAFSKInterval = 0;

  if (flightStatus == ASCENDING || flightStatus == DESCENDING) {
    beaconAFSKInterval = afskConfig.flightInterval;
  } else {
    beaconAFSKInterval = afskConfig.groundInterval;
  }

  if (batteryVoltage < powerConfig.lowPowerVoltage && flightStatus == LANDED) {
    beaconAFSKInterval = powerConfig.lowPowerInterval;
  }

  if (afskConfig.enabled && millis() - lastAFSKTXPacketTime > beaconAFSKInterval) {
    if (gpsFix) {
      sendAFSKLocation();
      lastAFSKTXPacketTime = millis();
      freeMem();
    }
  }
}

void sendAFSKLocation() {
#if defined(DEVMODE)
  SerialUSB.println(F("Location (AFSK) sending with comment"));
#endif

  SI4463_ON;
  delay(50);
  bool si4463Ready = false;
  for (uint8_t attempt = 0; attempt < 3 && !si4463Ready; attempt++) {
    if (si4463.init()) {
      si4463Ready = true;
    } else {
      SerialUSB.print("Si4463 init retry "); SerialUSB.println(attempt + 1);
      delay(100);
    }
  }
  if (!si4463Ready) {
    SerialUSB.println("Si4463 init fail!");
    SI4463_OFF;
  } else {
#if defined(DEVMODE)
    SerialUSB.println("Si4463 Init OK");
#endif
    ledBlink(true);
    si4463.setFrequency(afskConfig.frequency + afskConfig.freqCorrection);
    si4463.setModemOOK();
    si4463.enterTxMode();
    analogWrite(A0, 128);

    char telemetry_buff[100];
    int telemetryLen = formatTelemetryData(telemetry_buff, sizeof(telemetry_buff), afskConfig.comment);

    // APRS PRECISION AND DATUM OPTION http://www.aprs.org/aprs12/datum.txt
    if (afskConfig.sendEnhancedPrecision && getAltitudeFeet() < 10000 && (size_t)telemetryLen < sizeof(telemetry_buff) - 7) {
      char dao[3];
      updatePositionAFSK(1, dao);
      sprintf(telemetry_buff + telemetryLen, " !w%s!", dao);
    } else {
      updatePositionAFSK(0, NULL);
    }

    SerialUSB.print(F("AFSK: "));
    SerialUSB.print(telemetry_buff);

    APRS_sendLoc(telemetry_buff);

    delay(10);
    si4463.enterStandbyMode();
    SI4463_OFF;

    SerialUSB.println(F(" ...transmitted"));
    ledBlink(false);
    txCount++;
  }
}

/**
 * Format common telemetry data (course/speed, altitude, sensors) into a char buffer.
 * Used by both AFSK and LoRa beacon functions to avoid code duplication.
 *
 * @param buf       Output buffer
 * @param bufLen    Buffer size
 * @param comment   Additional comment to append
 * @return          Number of characters written
 */
int formatTelemetryData(char *buf, size_t bufLen, const String &comment) {
  uint16_t heading = (uint16_t)getCourseDeg();
  uint16_t speed = (uint16_t)getSpeedKnots();
  double altitude = getAltitudeFeet();
  uint16_t satelliteCount = getSatellites();

  float temperature = myBME.readTempC();
  float humidity = myBME.readFloatHumidity();
  float pressure = myBME.readFloatPressure() / 100.0f; // Pa to hPa

  char voltage_str[8];
  dtostrf(batteryVoltage, 5, 2, voltage_str);
  voltage_str[5] = '\0';

  char humidity_str[8];
  dtostrf(humidity, 5, 2, humidity_str);
  humidity_str[5] = '\0';

  char pressure_str[10];
  dtostrf(pressure, 7, 2, pressure_str);
  pressure_str[7] = '\0';

  int written;
  if (altitude > 0) {
    written = snprintf(buf, bufLen, "%03d/%03d/A=%06lu %uTXC %sC %%%s %shPa %sV %uS %s",
      heading, speed, (long)altitude,
      txCount, String(temperature).c_str(),
      humidity_str, pressure_str, voltage_str,
      satelliteCount, comment.c_str());
  } else {
    written = snprintf(buf, bufLen, "%03d/%03d/A=%06ld %uTXC %sC %%%s %shPa %sV %uS %s",
      heading, speed, (long)altitude,
      txCount, String(temperature).c_str(),
      humidity_str, pressure_str, voltage_str,
      satelliteCount, comment.c_str());
  }

  if (written < 0 || (size_t)written >= bufLen) {
    written = bufLen - 1;
  }
  return written;
}

void checkGPSStatus() {
  updateGPSData();
  gpsFix = gps.location.isValid() && (gps.satellites.isValid()) && (gps.satellites.value() > 3);
  if (gpsFix) {
    ledBlink(false);
  } else {
    gpsSearchLedBlink();
  }
#if defined(DEVMODE)
  printGPSandSensorData();
#endif
}

float getCurrentAltitudeFeet() {
  float currentAltitude = 0;
  if (gpsFix) {
    currentAltitude = gps.altitude.feet();
  } else {
    currentAltitude = myBME.readFloatAltitudeFeet();
  }
  return currentAltitude;
}

void checkFlightStatus() {

  if (gpsFix && (millis() - altitudeLastProcessedTimeStamp) > flightConfig.altitudeCheckInterval) {
    float currentAltitude = getCurrentAltitudeFeet();
    hab_ma_update(&altitudeAvgShort, currentAltitude);
    hab_ma_update(&altitudeAvgLong, currentAltitude);

    if (altitudeAvgLong.bufferFull) {

      // Pre-launch: once enough altitude samples are collected, capture the launch altitude and arm launch detection
      if (flightStatus == INITIALIZING && launchAltitudeAvg == 0) {
        launchAltitudeAvg = altitudeAvgLong.average;
        flightStatus = READY_TO_LAUNCH;
        SerialUSB.print("Launch Altitude: "); SerialUSB.print(launchAltitudeAvg); SerialUSB.println("feet");
        SerialUSB.print("FLIGHT STATUS: READY_TO_LAUNCH - "); SerialUSB.println(flightStatus);
      }

      // Trigger ASCENDING if delta exceeds threshold OR altitude exceeds 10000 feet (slow ascent fallback)
      if (flightStatus == READY_TO_LAUNCH && ((altitudeAvgShort.average - altitudeAvgLong.average > flightConfig.deltaAscDescTrigger) || altitudeAvgShort.average > 10000.f)) {
        flightStatus = ASCENDING;
        SerialUSB.print("FLIGHT STATUS: ASCENDING - "); SerialUSB.println(flightStatus);
      }

      //checking if balloon is popped and descending
      if (flightStatus == ASCENDING && (altitudeAvgLong.average - altitudeAvgShort.average > flightConfig.deltaAscDescTrigger) && altitudeAvgShort.maxAverage > 10000.f) {
        flightStatus = DESCENDING;
        SerialUSB.print("FLIGHT STATUS: DESCENDING - "); SerialUSB.println(flightStatus);
      }

      //checking if balloon is landed
      if (flightStatus == DESCENDING && (altitudeAvgShort.average >= altitudeAvgLong.average)) {
        flightStatus = LANDED;
        SerialUSB.print("FLIGHT STATUS: LANDED - "); SerialUSB.println(flightStatus);
      }

      if (flightConfig.predictLanding) {
        lp_update(&predictor, (float)getLatitude(), (float)getLongitude(), (float)getAltitudeFeet(), millis());
        if (lp_has_predictions(&predictor) && getAltitudeFeet() < 16404 && flightStatus == DESCENDING) {
          char pred_buf[100];
          lp_format_aprs(&predictor, pred_buf, sizeof(pred_buf));
          SerialUSB.println(F("Sending LoRa APRS status message..."));

          String aprsMessage = String(loraConfig.header);
          String path = "";
          if (strlen(loraConfig.wide) > 0) {
            path = "," + String(loraConfig.wide);
          }
          String loraCallSign = String(loraConfig.callSign);
          aprsMessage += loraCallSign + ">APLIGP" + path + ":>"+ String(pred_buf);
          sendLoRaAPRSMessage(aprsMessage);
        }
      }
    } else {
      SerialUSB.print("Collecting sample altitude data for launch detection: ");SerialUSB.print(altitudeAvgLong.index);SerialUSB.print("/");SerialUSB.println(ALTITUDE_SAMPLES_LONG);
    }

    altitudeLastProcessedTimeStamp = millis();

    #if defined(DEVMODE)
        SerialUSB.print("CurAltitude: "); SerialUSB.println(currentAltitude);
        SerialUSB.print("AltAvgShort: "); SerialUSB.println(altitudeAvgShort.average);
        SerialUSB.print("AltAvgLong: "); SerialUSB.println(altitudeAvgLong.average);
        SerialUSB.print("MaxAltitudeAvg: "); SerialUSB.println(altitudeAvgShort.maxAverage);
        SerialUSB.print("FLIGHT STATUS: "); SerialUSB.println(flightStatus);
        SerialUSB.println("-----------------------");
    #endif

  }
}

void soundBuzzer() {
  BUZZER_ON;
  delay(soundBuzzerTimeON);
  BUZZER_OFF;
  delay(soundBuzzerTimeOFF);
}

void loopBuzzer() {
  uint32_t buzzerInterval;

  if (batteryVoltage < powerConfig.lowPowerVoltage) {
    buzzerInterval = powerConfig.lowPowerInterval;
  } else {
    buzzerInterval = buzzerSoundInterval;
  }

  if (enableBuzzerInLoop && (flightStatus == LANDED) && ((millis() - lastBuzzerSounded) > buzzerInterval)) {
    for (uint8_t i = 0; i < soundBuzzerCount; i++) {
      soundBuzzer();
    }
    lastBuzzerSounded = millis();
  }
}

void loopFlashLED() {
  bool flashLEDEnabled = false;
  if(enableFlashLEDBlinkInLoop) {
      if (doNotBlinkDuringDay && !isNight()) {
        flashLEDEnabled = false;
      } else {
        flashLEDEnabled = true;
      }
  }


  uint32_t blinkInterval;

  if (batteryVoltage < powerConfig.lowPowerVoltage) {
    blinkInterval = powerConfig.lowPowerInterval;
  } else {
    blinkInterval = flashLEDBlinkInterval;
  }

  bool altitudeCheck = getCurrentAltitudeFeet() < maxFlashLEDAltitude;
  bool flightStatusCheck = (flightStatus >= flightStatusLevelForLEDBlink);
  bool intervalCheck = (millis() - lastFlashLEDBlinked) > blinkInterval;

  if (flashLEDEnabled && altitudeCheck && flightStatusCheck && intervalCheck) {
    for (uint8_t i = 0; i < flashLEDBlinkCount; i++) {
      blinkFlashLED();
    }
    lastFlashLEDBlinked = millis();
  }
}

void blinkFlashLED() {
  FLASH_LED_ON;
  delay(flashLEDBlinkTimeON);
  FLASH_LED_OFF;
  delay(flashLEDBlinkTimeOFF);
}

void sendAFSKStatus() {
  SerialUSB.println(F("Sending AFSK APRS status message..."));
  SI4463_ON;
  delay(50);
  bool si4463Ready = false;
  for (uint8_t attempt = 0; attempt < 3 && !si4463Ready; attempt++) {
    if (si4463.init()) {
      si4463Ready = true;
    } else {
      SerialUSB.print("Si4463 init retry "); SerialUSB.println(attempt + 1);
      delay(100);
    }
  }
  if (!si4463Ready) {
    SerialUSB.println("Si4463 init fail!");
    SI4463_OFF;
  } else {
#if defined(DEVMODE)
    SerialUSB.println("Si4463 Init OK");
#endif
    ledBlink(true);
    si4463.setFrequency(afskConfig.frequency + afskConfig.freqCorrection);
    si4463.setModemOOK();
    si4463.enterTxMode();
    analogWrite(A0, 128);
    delay(500);

    APRS_sendStatus(afskConfig.statusMessage);

    delay(10);
    analogWrite(A0, 255); //Power Save
    si4463.enterStandbyMode();
    SI4463_OFF;
    SerialUSB.print(F("AFSK Status sent (Freq: "));
    SerialUSB.print(afskConfig.frequency);
    SerialUSB.print(F(") - "));
    SerialUSB.println(txCount);
    ledBlink(false);
    txCount++;
  }
}

void updatePositionAFSK(int high_precision, char *dao) {
  // Convert and set latitude NMEA string Degree Minute Hundreths of minutes ddmm.hh[S,N].
  char latStr[10];
  RawDegrees rawDeg = gps.location.rawLat();
  uint32_t min_nnnnn;
  char lat_dao = 0;
  min_nnnnn = rawDeg.billionths * 0.006;
  if ( ((min_nnnnn / (high_precision ? 1 : 100)) % 10) >= 5 && min_nnnnn < (6000000 - ((high_precision ? 1 : 100) * 5)) ) {
    min_nnnnn = min_nnnnn + (high_precision ? 1 : 100) * 5;
  }
  sprintf(latStr, "%02u%02u.%02u%c", (unsigned int)(rawDeg.deg % 100), (unsigned int)((min_nnnnn / 100000) % 100), (unsigned int)((min_nnnnn / 1000) % 100), rawDeg.negative ? 'S' : 'N');
  if (dao)
    dao[0] = (char)((min_nnnnn % 1000) / 11) + 33;
  APRS_setLat(latStr);

  // Convert and set longitude NMEA string Degree Minute Hundreths of minutes ddmm.hh[E,W].
  char lonStr[10];
  rawDeg = gps.location.rawLng();
  min_nnnnn = rawDeg.billionths * 0.006;
  if ( ((min_nnnnn / (high_precision ? 1 : 100)) % 10) >= 5 && min_nnnnn < (6000000 - ((high_precision ? 1 : 100) * 5)) ) {
    min_nnnnn = min_nnnnn + (high_precision ? 1 : 100) * 5;
  }
  sprintf(lonStr, "%03u%02u.%02u%c", (unsigned int)(rawDeg.deg % 1000), (unsigned int)((min_nnnnn / 100000) % 100), (unsigned int)((min_nnnnn / 1000) % 100), rawDeg.negative ? 'W' : 'E');
  if (dao) {
    dao[1] = (char)((min_nnnnn % 1000) / 11) + 33;
    dao[2] = 0;
  }

  APRS_setLon(lonStr);
  APRS_setTimeStamp(gps.time.hour(), gps.time.minute(), gps.time.second());
}

void sendLoRaAPRSMessage(String aprsMessage) {
  radio.clearPacketReceivedAction();
  receivedFlag = false;
  SerialUSB.print("LoRa: " + aprsMessage);
  ledBlink(true);
  int state = radio.transmit(aprsMessage);
  if (state == RADIOLIB_ERR_NONE) {
    SerialUSB.println(F(" ...transmitted"));
  } else if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
    SerialUSB.println(F(" The supplied packet was longer than 256 bytes!"));
  } else if (state == RADIOLIB_ERR_TX_TIMEOUT) {
    SerialUSB.println(F(" Timeout occured while transmitting packet!"));
  } else {
    SerialUSB.print(F(" failed, code "));
    SerialUSB.println(state);
  }
  ledBlink(false);
  enableRX();
}

String getTrackerLocationLoRaAPRSMessage(bool hasGPSFix) {
  String message;
  String path = "";
  if (strlen(loraConfig.wide) > 0) {
    path = "," + String(loraConfig.wide);
  }

  String loraCallSign = String(loraConfig.callSign);

  if (hasGPSFix) {
    message += loraCallSign + ">APLIGP" + path + ":/";

    char timestamp[8];
    hab_encodeHMSTimestamp(getHour(), getMinute(), getSecond(), timestamp, sizeof(timestamp));
    message += String(timestamp);

    char latitude[9];
    char longitude[10];
    hab_createLatForAPRS(getLatitude(), latitude, sizeof(latitude));
    hab_createLongForAPRS(getLongitude(), longitude, sizeof(longitude));
    message += String(latitude) + String(loraConfig.symbolTable) + String(longitude) + String(loraConfig.symbolCode);

    // Telemetry
    char telemetry_buff[100];
    String commentWithFS = String(loraConfig.comment) + " FS:" + String(flightStatus);
    if (cutDownConfig.enabled) {
      commentWithFS += " CDC" + String(cutDownCommandCount);
    }
    formatTelemetryData(telemetry_buff, sizeof(telemetry_buff), commentWithFS);
    message += String(telemetry_buff);

  } else {
    message += loraCallSign + ">APLIGP" + path + ":>";
    // Status-only telemetry
    char telemetry_buff[100];
    String commentWithFS = String(loraConfig.comment) + " FS:" + String(flightStatus);
    if (cutDownConfig.enabled) {
      commentWithFS += " CDC" + String(cutDownCommandCount);
    }
    formatTelemetryData(telemetry_buff, sizeof(telemetry_buff), commentWithFS);
    message += String(telemetry_buff);
  }

  return message;
}

void sendLoRaLocationMessage(bool hasGPSFix) {
  String aprsMessage = String(loraConfig.header) + getTrackerLocationLoRaAPRSMessage(hasGPSFix);
  sendLoRaAPRSMessage(aprsMessage);
  txCount++;
}

void sendLoRaStatusMessage() {
  SerialUSB.println(F("Sending LoRa APRS status message..."));
  String aprsMessage = String(loraConfig.header) + getTrackerStatusLoRaAPRSMessage();
  sendLoRaAPRSMessage(aprsMessage);
  txCount++;
}

String getTrackerStatusLoRaAPRSMessage() {
  String message;
  String path = "";
  if (strlen(loraConfig.wide) > 0) {
    path = "," + String(loraConfig.wide);
  }
  String loraCallSign = String(loraConfig.callSign);
  message += loraCallSign + ">APLIGP" + path + ":>" + String(loraConfig.statusMessage) + " " + readBatt() + "V";
  return message;
}

void sendCutDownStatusMessage() {
  String aprsMessage = String(loraConfig.header) + getCutDownStatusAPRSMessage();
  sendLoRaAPRSMessage(aprsMessage);
  txCount++;
}

String getCutDownStatusAPRSMessage() {
  String message;
  String path = "";
  if (strlen(loraConfig.wide) > 0) {
    path = "," + String(loraConfig.wide);
  }
  String loraCallSign = String(loraConfig.callSign);
  message += loraCallSign + ">APLIGP" + path + ":>CUT DOWN EXECUTED";
  return message;
}

void setupBME280() {
  SerialUSB.println(F("BME280 Setup"));
  myBME.setI2CAddress(0x76);
  if (myBME.beginI2C() == false) {
    SerialUSB.println("The BME280 did not respond. Please check wiring.");
  }
}

void setupGPS() {
  SerialUSB.println(F("GPS setup"));
  Serial.begin(9600);

  for (int i = 0; i < 5; i++) {
    if (setGPSBalloonMode()) {
      SerialUSB.println("Quectel GPS High Altitude Balloon Mode Enabled!!!");
      break;
    } else {
      SerialUSB.println("Balloon Mode Enable Attempt: " + String(i + 1));
    }
  }

  SerialUSB.println(F("Searching for GPS fix..."));
  altitudeLastProcessedTimeStamp = millis();
}

void updateGPSData() {
  while (Serial.available() > 0) {
    gps.encode(Serial.read());
  }

  if ((millis() - gpsLastProcessedTimeStamp) > 10000 && gps.charsProcessed() < 10) {
    SerialUSB.print("No GPS detected: check wiring.");
    while (true);
  }
  gpsLastProcessedTimeStamp = millis();
}

void parseRXPacket() {
  String packet;
  int state = radio.readData(packet);

  if (state == RADIOLIB_ERR_NONE) {
    if (packet.length() > 0) {
      String loraCallSign = String(loraConfig.callSign);
      if ((packet.substring(0, 3) == "\x3c\xff\x01") && (packet.indexOf("TCPIP") == -1) && (packet.indexOf("NOGATE") == -1)) {
        String sender = packet.substring(3, packet.indexOf(">"));
        if ((packet.indexOf(loraCallSign + "_" + String(cutDownConfig.secret)) > 0) && (loraCallSign != sender) && !cutDownInProgress) {
          SerialUSB.println(F("Cut Down Command Recieved..."));
          CUTDOWN_ON;
          ++cutDownCommandCount;
          SerialUSB.println(F("Cut Down Command Started..."));
          lastCutDown = millis();
          sendCutDownStatusMessage();
          cutDownInProgress = true;
        }
      }
    }
  } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
    SerialUSB.println(F("CRC error!"));
  } else {
    SerialUSB.print(F("failed, code "));
    SerialUSB.println(state);
  }
}

void enableRX() {
  if (cutDownConfig.enabled) {
    radio.setPacketReceivedAction(setFlag);

    SerialUSB.print(F("[SX1268] Starting to listen ... "));
    int state = radio.startReceive();
    if (state == RADIOLIB_ERR_NONE) {
      SerialUSB.println(F("success!"));
    } else {
      SerialUSB.print(F("failed, code "));
      SerialUSB.println(state);
    }
  }
}

void printGPSandSensorData() {
  SerialUSB.print(F("Sats: "));
  if (gps.satellites.isValid()) {
    SerialUSB.print(gps.satellites.value());
  } else {
    SerialUSB.print(F("INVALID"));
  }

  SerialUSB.print(F(" Location: "));
  if (gps.location.isValid()) {
    SerialUSB.print(gps.location.lat(), 6);
    SerialUSB.print(F(","));
    SerialUSB.print(gps.location.lng(), 6);
  } else {
    SerialUSB.print(F("INVALID"));
  }

  SerialUSB.print(F(" Altitude: "));
  if (gps.altitude.isValid()) {
    SerialUSB.print(gps.altitude.feet()); SerialUSB.print(F("feet"));
  } else {
    SerialUSB.print(F("INVALID"));
  }

  SerialUSB.print(F("  Date/Time: "));
  if (gps.date.isValid()) {
    SerialUSB.print(gps.date.month());
    SerialUSB.print(F("/"));
    SerialUSB.print(gps.date.day());
    SerialUSB.print(F("/"));
    SerialUSB.print(gps.date.year());
  } else {
    SerialUSB.print(F("INVALID"));
  }

  SerialUSB.print(F(" "));
  if (gps.time.isValid()) {
    if (gps.time.hour() < 10) SerialUSB.print(F("0"));
    SerialUSB.print(gps.time.hour());
    SerialUSB.print(F(":"));
    if (gps.time.minute() < 10) SerialUSB.print(F("0"));
    SerialUSB.print(gps.time.minute());
    SerialUSB.print(F(":"));
    if (gps.time.second() < 10) SerialUSB.print(F("0"));
    SerialUSB.print(gps.time.second());
    SerialUSB.print(F("."));
    if (gps.time.centisecond() < 10) SerialUSB.print(F("0"));
    SerialUSB.print(gps.time.centisecond());
  } else {
    SerialUSB.print(F("INVALID"));
  }

  SerialUSB.print(" Temp: ");
  SerialUSB.print(myBME.readTempC(), 2);
  SerialUSB.print("C");

  SerialUSB.print(" Humidity: ");
  SerialUSB.print(myBME.readFloatHumidity(), 2);
  SerialUSB.print("%");

  SerialUSB.print(" Pressure: ");
  SerialUSB.print(myBME.readFloatPressure() / 100.0);
  SerialUSB.print("hPa ");

  SerialUSB.print(" Barometric Alt: ");
  SerialUSB.print(myBME.readFloatAltitudeFeet());
  SerialUSB.print("feet ");

  SerialUSB.print(batteryVoltage);
  SerialUSB.print("V");

  SerialUSB.println();
  delay(500);
}

float readBatt() {
  float R1 = 560000.0; // 560K
  float R2 = 100000.0; // 100K
  float value = 0.0f;
  uint8_t retries = 0;
  const uint8_t MAX_RETRIES = 10;
  do {
    value = analogRead(BATT_PIN);
    value += analogRead(BATT_PIN);
    value += analogRead(BATT_PIN);
    value = value / 3.0f;
    value = (value * 3.3) / 1024.0f;
    value = value / (R2 / (R1 + R2));
    retries++;
  } while (value > 20.0 && retries < MAX_RETRIES);
  if (retries >= MAX_RETRIES) return 0.0f;
  return value;
}

void freeMem() {
#if defined(DEVMODE)
  SerialUSB.print(F("Free RAM: ")); SerialUSB.print(freeMemory(), DEC); SerialUSB.println(F(" byte"));
#endif
}

void setupAFSK() {
  SerialUSB.println(F("AFSK Setup"));
  analogWrite(A0, 0);
  pinMode(SI446X_SDN_PIN, OUTPUT);

  APRS_init();
  APRS_setCallsign(afskConfig.callSign, afskConfig.callNumber);
  APRS_setDestination("APLIGA", 0);
  APRS_setPath1("WIDE1", afskConfig.wide1);
  APRS_setPath2("WIDE2", afskConfig.wide2);
  APRS_useAlternateSymbolTable(afskConfig.alternateSymbolTable);
  APRS_setSymbol(afskConfig.symbol);
  APRS_setPathSize(afskConfig.pathSize);
}

void setupLoRa() {
  SerialUSB.println(F("LoRa Setup"));
  SerialUSB.print(F("[SX1268] LoRa Radio Module Initializing ... "));
  int state = radio.begin();

  if (state == RADIOLIB_ERR_NONE) {
    SerialUSB.println(F("success!"));
  } else {
    SerialUSB.print(F("failed, code "));
    SerialUSB.println(state);
    while (true);
  }

  if (radio.setFrequency(loraConfig.frequency, true) == RADIOLIB_ERR_INVALID_FREQUENCY) {
    SerialUSB.println(F("Selected frequency is invalid for this module!"));
    while (true);
  }

  if (radio.setBandwidth(loraConfig.bandwidth) == RADIOLIB_ERR_INVALID_BANDWIDTH) {
    SerialUSB.println(F("Selected bandwidth is invalid for this module!"));
    while (true);
  }

  if (radio.setSpreadingFactor(loraConfig.spreadingFactor) == RADIOLIB_ERR_INVALID_SPREADING_FACTOR) {
    SerialUSB.println(F("Selected spreading factor is invalid for this module!"));
    while (true);
  }

  if (radio.setCodingRate(loraConfig.codingRate) == RADIOLIB_ERR_INVALID_CODING_RATE) {
    SerialUSB.println(F("Selected coding rate is invalid for this module!"));
    while (true);
  }

  if (radio.setSyncWord(RADIOLIB_SX126X_SYNC_WORD_PRIVATE) != RADIOLIB_ERR_NONE) {
    SerialUSB.println(F("Unable to set sync word!"));
    while (true);
  }

  if (radio.setCurrentLimit(140) == RADIOLIB_ERR_INVALID_CURRENT_LIMIT) {
    SerialUSB.println(F("Selected current limit is invalid for this module!"));
    while (true);
  }

  if (radio.setOutputPower(loraConfig.outputPower) == RADIOLIB_ERR_INVALID_OUTPUT_POWER) {
    SerialUSB.println(F("Selected output power is invalid for this module!"));
    while (true);
  }

  if (radio.setPreambleLength(loraConfig.preambleLength) == RADIOLIB_ERR_INVALID_PREAMBLE_LENGTH) {
    SerialUSB.println(F("Selected preamble length is invalid for this module!"));
    while (true);
  }

  if (radio.setCRC(loraConfig.crc) == RADIOLIB_ERR_INVALID_CRC_CONFIGURATION) {
    SerialUSB.println(F("Selected CRC is invalid for this module!"));
    while (true);
  }

  if (loraConfig.enabled) {
    radio.setRxBoostedGainMode(true);
  } else {
    radio.sleep();
  }

  SerialUSB.println(F("All settings succesfully changed!"));
}

void setGPSStandbyMode(bool enable) {
  const char *command;

  if (enable) {
    command = "$PMTK161,0*28\r\n";
    gpsStandbyMode = true;
    gpsFix = false;
  } else {
    command = "$PMTK010,001*2E<CR><LF>";
    gpsStandbyMode = false;
  }

  const unsigned long timeout = 1000;
  unsigned long startTime;

  Serial.print(command);

  startTime = millis();
  String response = "";

  while (millis() - startTime < timeout) {
    while (Serial.available()) {
      char c = Serial.read();
      response += c;
    }
  }
  delay(500);
}

bool setGPSBalloonMode() {
  const char *command = "$PMTK886,3*2B\r\n";
  const unsigned long timeout = 1000;
  unsigned long startTime;

  Serial.print(command);

  startTime = millis();
  String response = "";

  while (millis() - startTime < timeout) {
    while (Serial.available()) {
      char c = Serial.read();
      response += c;

      if (response.indexOf("PMTK001,886,3") != -1) {
        return true;
      }
    }
  }

  return false;
}

void gpsSearchLedBlink() {
  if (gpsFixSearchLed) {
    ledBlink(true);
    gpsFixSearchLed = false;
  } else {
    ledBlink(false);
    gpsFixSearchLed = true;
  }
  delay(500);
}

void ledBlink(bool enabled) {
  if (enabled) {
    digitalWrite(STATUS_LED_PIN, HIGH);
  } else {
    digitalWrite(STATUS_LED_PIN, LOW);
  }
}

bool isNight() {
  bool night = false;

  bool validData = gps.location.isValid() && gps.time.isValid() && gps.date.isValid();

  if (validData) {
    double latitude = getLatitude();
    double longitude = getLongitude();
    double altitude = getAltitudeMeters();

    int currentHour = getHour();
    int currentMinute = getMinute();
    int dayOfYear = getDayOfYear();

    LocationTime loc = {latitude, longitude, altitude, currentHour, currentMinute, dayOfYear};
    DayNightStatus status = hab_analyzeDayNight(loc);

    if (status.isDay && status.sunElevation > 0) {
      // Day
    } else if (!status.isDay && status.sunElevation < 1.0) {
      night = true;
    } else {
      night = true; // Dawn/Dusk
    }
  }

  return night;
}
