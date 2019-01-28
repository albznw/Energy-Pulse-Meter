#include <Arduino.h>

#include <ESP8266WiFi.h>          
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include <PubSubClient.h>
#include <TaskScheduler.h>
#include <ESP8266WiFi.h>
#include <EEPROM.h>

#include <debugHeaders.h>


#include "secrets.h" // is needed for wifi, ota and mqtt
#include "eepromReadWrite.hpp"

/******************************** Variables ***********************************/
//Number of pulses, used to measure energy.
unsigned long pulseCount = 0;

//Used to measure power.
unsigned long pulseTime, lastPulseTime;

//power and energy
double power, elapsedkWh;

//Number of pulses per wh - found or set on the meter.
int ppwh = 1; //1000 pulses/kwh = 1 pulse per wh

// Used for status leds
unsigned long onMs, offMs; 
int colors, blinkTimes;

/**************************** General - Settings ******************************/
#define OTA_HOSTNAME                "EnergyPulseMeter" // Leave empty for esp8266-[ChipID]
#define WIFI_MANAGER_STATION_NAME   "EnergyPulseMeter" // Leave empty for auto generated name ESP + ChipID

#define STATUS_LED_BLUE_PIN         D6 // Blue status led pin
#define STATUS_LED_GREEN_PIN        D7 // Green status led pin
#define STATUS_LED_RED_PIN          D8 // Red status led pin
#define SENSOR_ONE_PIN              D1 // Pulse sensor one
#define SENSOR_TWO_PIN              D2 // Pulse sensor two
#define SENSOR_ONE_INDICATOR_PIN    D5 // Sensor one indicator led 
#define SENSOR_TWO_INDICATOR_PIN    D0 // Sensor two indicator led

bool OTA_ON = true; // Turn on OTA

/****************************** MQTT - Settings *******************************/
// Connection things is found in Secret.h
#define MQTTClientId        "logitech_z906"
#define MQTTCategory        "speaker"

#define ClientRoot          MQTTCategory "/" MQTTClientId

// Some examples on how the routes should be
#define CommandTopic        ClientRoot "/cmnd/json"
#define StateTopic          ClientRoot "/state/json"
#define DebugTopic          ClientRoot "/debug"
#define WillTopic           ClientRoot "/will"
#define WillQoS             0
#define WillRetain          false
const char* willMessage = MQTTClientId " has disconnected...";

#define FirstMessage        "I communicate via JSON!"

WiFiClient wificlient;  // is needed for the mqtt client
PubSubClient mqttclient;

/*********************************** Tasks ************************************/
Scheduler taskManager;
#define HALF_MINUTE 30000UL

// Define relevant functions
void saveToEEPROM();
void checkWifiStatus();
void blinkStatusLed();
void disableBlinkStatusLed();
void sensorOneLed();

Task tSaveToEEPROM(TASK_MINUTE, TASK_FOREVER, &saveToEEPROM, &taskManager);
Task tCheckWifiStatus(TASK_MINUTE, TASK_FOREVER, &checkWifiStatus, &taskManager);
Task tBlink(200, 3, &blinkStatusLed, &taskManager, false, NULL, &disableBlinkStatusLed);
Task tSensorOneLed(150, TASK_FOREVER, &sensorOneLed, &taskManager);

/***************************** EEPROM ADDRESSES *******************************/
// Only writing 32 bit information (4 bytes)
#define ELAPSED_KWH_ADDR  1
#define PULSE_COUNT_ADDR  5

/*********************************** MISC *************************************/
#define BLUE    0b001
#define RED     0b010
#define GREEN   0b100

void setupOTA() {
  TraceFunc("Initializing...\n");
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  if (OTA_HOSTNAME != "") {
    ArduinoOTA.setHostname(OTA_HOSTNAME);
  }

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
    digitalWrite(STATUS_LED_BLUE_PIN, LOW);
    digitalWrite(STATUS_LED_GREEN_PIN, HIGH);
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    if(progress % 5 == 0)
      digitalWrite(STATUS_LED_BLUE_PIN, !digitalRead(STATUS_LED_BLUE_PIN));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    digitalWrite(STATUS_LED_RED_PIN, HIGH);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  TraceFunc("Initializing done.\n");
}

void setupWifiManager() {
  TraceFunc("Initializing\n");
  // WiFiManager
  // Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  // reset saved settings
  // wifiManager.resetSettings();

  wifiManager.setConfigPortalTimeout(180);

  // fetches ssid and pass from eeprom and tries to connect
  // if it does not connect it starts an access point with the specified name
  // and goes into a blocking loop awaiting configuration
  if (WIFI_MANAGER_STATION_NAME == "") {
    // use this for auto generated name ESP + ChipID
    wifiManager.autoConnect();
  } else {
    wifiManager.autoConnect(WIFI_MANAGER_STATION_NAME);
  }

  if(WiFi.isConnected()) {
    Serial.print("WiFi Connected: ");
    Serial.println(WiFi.localIP());
  } else {
    ESP.restart();
  }

  TraceFunc("Done.\n");
}

void saveToEEPROM() {
  Log("Saving to EEPROM\n");
  EEPROMWritelong(ELAPSED_KWH_ADDR, elapsedkWh);
  EEPROMWritelong(PULSE_COUNT_ADDR, pulseCount);
}

void resetEEPROM() {
  EEPROMWritelong(ELAPSED_KWH_ADDR, 0);
  EEPROMWritelong(PULSE_COUNT_ADDR, 0);
}

void readFromEEPROM() {
  elapsedkWh = (double) EEPROMReadlong(ELAPSED_KWH_ADDR);
  pulseCount = (unsigned long) EEPROMReadlong(PULSE_COUNT_ADDR);
}

void reconnectWifiCallback() {

}

void disableBlinkStatusLed() {
  digitalWrite(STATUS_LED_BLUE_PIN, LOW);
  digitalWrite(STATUS_LED_GREEN_PIN, LOW);
  digitalWrite(STATUS_LED_RED_PIN, LOW);
} 

void blinkStatusLed() {
  static unsigned long lastChanged = 0;
  static int counter = 0;
  static bool ledOn = false;
  
  unsigned long currentMs = millis();

  if(ledOn) {
    // leds are on
    if(currentMs - lastChanged > onMs) {
      if((colors & RED) == RED) 
        digitalWrite(STATUS_LED_RED_PIN, LOW);
      if((colors & BLUE) == BLUE) 
        digitalWrite(STATUS_LED_BLUE_PIN, LOW);
      if((colors & GREEN) == GREEN) 
        digitalWrite(STATUS_LED_GREEN_PIN, LOW);

      ledOn = false;
      lastChanged = currentMs;
      counter++;
    }
  } else {
    // leds are off
    if(currentMs -lastChanged > offMs) {
      if((colors & RED) == RED) 
        digitalWrite(STATUS_LED_RED_PIN, HIGH);
      if((colors & BLUE) == BLUE) 
        digitalWrite(STATUS_LED_BLUE_PIN, HIGH);
      if((colors & GREEN) == GREEN) 
        digitalWrite(STATUS_LED_GREEN_PIN, HIGH);

      ledOn = true;
      lastChanged = currentMs;
    }
  }

  //Check if its time to terminate
  if(counter >= blinkTimes) {
    tBlink.disable();
  }
}

void setStatusLed(uint8_t _colors, long _onMs = 300, long _offMs = 200, int _blinkTimes = 0) {
  onMs = _onMs;
  offMs = _offMs;
  colors = _colors;
  blinkTimes = _blinkTimes;

  taskManager.addTask(tBlink);
  tBlink.setIterations(-1);
  tBlink.enable();
}

void checkWifiStatus() {
  if(!WiFi.isConnected()) {
    tCheckWifiStatus.setCallback(&reconnectWifiCallback);
  }
}

void turnOffSensorOneLed() {
  TraceFunc("Sensor one indicator led - OFF\n");
  digitalWrite(SENSOR_ONE_INDICATOR_PIN, LOW);
  tSensorOneLed.setCallback(sensorOneLed);
  tSensorOneLed.disable();
}

void sensorOneLed() {
  digitalWrite(SENSOR_ONE_INDICATOR_PIN, HIGH);
  TraceFunc("Sensor one indicator led - ON\n");
  tSensorOneLed.setCallback(turnOffSensorOneLed);
}

void sensorOneHandler() {
  static unsigned long lastTime = 0;
  digitalWrite(SENSOR_ONE_INDICATOR_PIN, HIGH);

  // used to measure time between pulses
  unsigned long currentTime = millis();

  if(lastTime == 0) {
    // This is the first pulse
    lastTime = currentTime;
    tSensorOneLed.enable();
    return;
  }

  pulseCount++;

  // calc power
  power = (3600000000.0 / (currentTime - lastTime)) / ppwh;

  // calc consumed kWh
  elapsedkWh = (1.0 * pulseCount / (ppwh * 1000)); //multiply by 1000 to convert pulses per wh to kwh

  //Print the values.
  Serial.print(power,4);
  Serial.print(" ");
  Serial.println(elapsedkWh,3);

  Log("Power: %.4f W\n", power);
  Log("Consumed: %.3f kWh\n", elapsedkWh);

  lastTime = currentTime;
  tSensorOneLed.enable();
}

void setup() {
  Serial.begin(115200);
  Serial.println("");

  // Initialize status leds
  pinMode(STATUS_LED_BLUE_PIN, OUTPUT);
  pinMode(STATUS_LED_GREEN_PIN, OUTPUT);
  pinMode(STATUS_LED_RED_PIN, OUTPUT);
  pinMode(SENSOR_ONE_INDICATOR_PIN, OUTPUT);
  pinMode(SENSOR_TWO_INDICATOR_PIN, OUTPUT);

  // Set them to LOW
  digitalWrite(STATUS_LED_BLUE_PIN, LOW);
  digitalWrite(STATUS_LED_GREEN_PIN, LOW);
  digitalWrite(STATUS_LED_RED_PIN, LOW);
  digitalWrite(SENSOR_ONE_INDICATOR_PIN, LOW);
  digitalWrite(SENSOR_TWO_INDICATOR_PIN, LOW);

  EEPROM.begin(512);
  // read old data from eeprom
  readFromEEPROM();

  pinMode(SENSOR_ONE_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(SENSOR_ONE_PIN), sensorOneHandler, RISING);

  setupWifiManager();
  setupOTA();

  tCheckWifiStatus.enable();
  tSaveToEEPROM.enable();
  Log("\nEnergy meter setup done\n\n");
}

void loop() {
  if(OTA_ON) {
    noInterrupts();
    ArduinoOTA.handle();
    interrupts();
  }

  taskManager.execute();
}