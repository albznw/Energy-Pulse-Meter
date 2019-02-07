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
#include <NTPClient.h>
#include <ArduinoJson.h>
#include <TimeLib.h>

#include <debugHeaders.h>

#include "secrets.h" // is needed for wifi, ota and mqtt
#include "eepromReadWrite.hpp"

/******************************** Variables ***********************************/
int currentDay, lastDay; // keep track of which day it is

//Number of pulses, used to measure energy.
unsigned long pulseCount = 0; // resets every day

//Used to measure power.
unsigned long lastPulseTime;

//power and energy
double power = 0;
double todaykWh = 0;
double yesterdaykWh = 0;
double totalkWh = 0;

//Number of pulses per wh - found or set on the meter.
int ppwh = 1; //1000 pulses/kwh = 1 pulse per wh

// Used for status leds
unsigned long onMs, offMs; 
int colors, blinkTimes;

#define SENSOR_COOLDOWN 20 // ms

/**************************** General - Settings ******************************/
#define OTA_HOSTNAME                "EnergyPulseMeter" // Leave empty for esp8266-[ChipID]
#define WIFI_MANAGER_STATION_NAME   "EnergyPulseMeter" // Leave empty for auto generated name ESP + ChipID

#define STATUS_LED_BLUE_PIN         D1 // Blue status led pin
#define STATUS_LED_GREEN_PIN        D2 // Green status led pin
#define STATUS_LED_RED_PIN          D7 // Red status led pin
#define SENSOR_ONE_PIN              D5 // Pulse sensor one
#define SENSOR_TWO_PIN              D6 // Pulse sensor two
#define SENSOR_ONE_INDICATOR_PIN    D3 // Sensor one indicator led 
#define SENSOR_TWO_INDICATOR_PIN    D0 // Sensor two indicator led

bool OTA_ON = true; // Turn on OTA

/****************************** MQTT - Settings *******************************/
// Connection things is found in secret.h
#define MQTTClientId        "energy_pulse_meter"
#define MQTTCategory        "sensor"

#define ClientRoot          MQTTCategory "/" MQTTClientId

// Some examples on how the routes should be
#define CommandTopic        ClientRoot "/cmnd/"
#define StateTopic          ClientRoot "/state/json"
#define TelemetryTopic      ClientRoot "/tele"
#define DebugTopic          ClientRoot "/debug"
#define WillTopic           ClientRoot "/will"
#define WillQoS             0
#define WillRetain          false
const char* willMessage = MQTTClientId " has disconnected...";

#define FirstMessage        "I communicate via JSON!"

WiFiClient wificlient;  // is needed for the mqtt client
PubSubClient mqttclient;

/*********************************** Tasks ************************************/
WiFiUDP ntpUDP;

// You can specify the time server pool and the offset (in seconds, can be
// changed later with setTimeOffset() ). Additionaly you can specify the
// update interval (in milliseconds, can be changed using setUpdateInterval() ).
NTPClient timeClient(ntpUDP, "se.pool.ntp.org", 3600, 60000);

/*********************************** Tasks ************************************/
Scheduler taskManager;
#define HALF_MINUTE 30000UL

// Define relevant functions
void saveToEEPROM();
void checkWifiStatus();
void blinkStatusLed();
void disableBlinkStatusLed();
void sensorOneLed();
void updateTime();
void sendTelemetry();

Task tSaveToEEPROM(TASK_MINUTE, TASK_FOREVER, &saveToEEPROM, &taskManager);
Task tCheckWifiStatus(TASK_MINUTE, TASK_FOREVER, &checkWifiStatus, &taskManager);
Task tBlink(200, 3, &blinkStatusLed, &taskManager, false, NULL, &disableBlinkStatusLed);
Task tSensorOneLed(150, TASK_FOREVER, &sensorOneLed, &taskManager);
Task tUpdateTime(TASK_MINUTE, TASK_FOREVER, &updateTime, &taskManager);
Task tSendTelemetry(HALF_MINUTE, TASK_FOREVER, &sendTelemetry, &taskManager);

/***************************** EEPROM ADDRESSES *******************************/
// Only writing 32 bit information (4 bytes)
#define PULSE_COUNT_ADDR    1
#define ELAPSED_KWH_ADDR    5
#define YESTERDAY_KWH_ADDR  9
#define TOTAL_KWH_ADDR      13

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
  Debugf("Saving to EEPROM\n");
  EEPROMWritelong( (long) PULSE_COUNT_ADDR, pulseCount);
  EEPROMWritelong( (long) ELAPSED_KWH_ADDR, todaykWh);
  EEPROMWritelong( (long) YESTERDAY_KWH_ADDR, yesterdaykWh);
  EEPROMWritelong( (long) TOTAL_KWH_ADDR, totalkWh);
}

void resetEEPROM() {
  EEPROMWritelong(PULSE_COUNT_ADDR, 0);
  EEPROMWritelong(ELAPSED_KWH_ADDR, 0);
  EEPROMWritelong(YESTERDAY_KWH_ADDR, 0);
  EEPROMWritelong(TOTAL_KWH_ADDR, 0);
}

void readFromEEPROM() {
  pulseCount = (unsigned long) EEPROMReadlong(PULSE_COUNT_ADDR);
  todaykWh = (double) EEPROMReadlong(ELAPSED_KWH_ADDR);
  yesterdaykWh = (double) EEPROMReadlong(YESTERDAY_KWH_ADDR);
  totalkWh = (double) EEPROMReadlong(TOTAL_KWH_ADDR);
}

void updateTime() {
  Debugf("Updating time\n");
  timeClient.update();
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

void reconnectWifiCallback() {

}

void checkWifiStatus() {
  if(!WiFi.isConnected()) {
    tCheckWifiStatus.setCallback(&reconnectWifiCallback);
  }
}

bool publishMQTT(const char* topic, const char* payload){
  String printString = "";
  bool returnBool = false;
  if(mqttclient.publish(topic, payload)) {
    returnBool = true;
    printString = String("[publishMQTT] '" + String(payload) + "' was sent sucessfully to: ");
  } else {
    returnBool = false;
    printString = String("[publishMQTT] ERROR (" + String(mqttclient.state()) + ") sending: '" + String(payload) + "' to: ");
  }
  printString += topic;
  Traceln(printString);
  return returnBool;
}

bool publishMQTT(const char* topic, String payload){
  return publishMQTT(topic, payload.c_str());
}

String payloadToString(byte* payload, int length) {
  char message_buff[length];
  int i = 0;
  for (i = 0; i < length; i++) {
      message_buff[i] = payload[i];
    }
  message_buff[i] = '\0';
  return String(message_buff);
}

void mqttCallback(char* topic, byte* payload, int length) {

  //convert topic to string to make it easier to work with
  String topicStr = topic;
  String payloadStr = payloadToString(payload, length);

  Traceln("[MQTT][callback] Callback update.");
  Traceln(String("[MQTT][callback] Topic: " + topicStr));

  if(topicStr.equals(CommandTopic)){
    if(payloadStr.equals("restart")) {
      ESP.restart();
    } else if(payloadStr.equals("you on?")) {
      publishMQTT(StateTopic, "Yes");
    } else if(payloadStr.equals("reset")) {
      resetEEPROM();
      readFromEEPROM();
    }
  }
}

/** Connects to the MQTT broker and subscribes to the topic */
bool connectMQTT() {
  Trace("[MQTT] Connecting to MQTT server...");
  while (!mqttclient.connected()) {
    //if connected, subscribe to the topic(s) we want to be notified about
    if(mqttclient.connect(MQTTClientId, MQTTUsername, MQTTPassword)) {
      Traceln("MTQQ Connected!");
      if (mqttclient.subscribe(CommandTopic))
        Trace("[MQTT] Sucessfully subscribed to %s\n", CommandTopic);
      publishMQTT(DebugTopic, FirstMessage);
      return true;
    }
    Trace("Failed to connect to MQTT Server, state: %d\n", mqttclient.state());
    return false;
  } 
}

/** Be sure to setup WIFI before running this method! */
void setupMQTT() {
  mqttclient = PubSubClient(Broker, Port, mqttCallback, wificlient);
  connectMQTT();
}

void turnOffSensorOneLed() {
  // TraceFunc("Sensor one indicator led - OFF\n");
  digitalWrite(SENSOR_ONE_INDICATOR_PIN, LOW);
  tSensorOneLed.setCallback(sensorOneLed);
  tSensorOneLed.disable();
}

void sensorOneLed() {
  digitalWrite(SENSOR_ONE_INDICATOR_PIN, HIGH);
  // TraceFunc("Sensor one indicator led - ON\n");
  tSensorOneLed.setCallback(turnOffSensorOneLed);
}

void sensorOneHandler() {
  noInterrupts();
  static bool firstRun = true;
  digitalWrite(SENSOR_ONE_INDICATOR_PIN, HIGH);

  // used to measure time between pulses
  unsigned long currentTime = millis();

  if(firstRun) {
    lastPulseTime = currentTime;
    firstRun = false;
    return;
  } else if(currentTime - lastPulseTime <= SENSOR_COOLDOWN) {
    // the LED on the meter have a tendency to flicker with approx 3-10 ms
    // between on-off. This makes sure we do not record them.
    return;
  }
  pulseCount++;

  // calc power
  power = (3600000.0 / (currentTime - lastPulseTime)) / ppwh;

  // calc consumed kWh
  todaykWh = (1.0 * pulseCount / (ppwh * 1000)); //multiply by 1000 to convert pulses per wh to kwh

  Log("%d\t%.4f W\t%.3f kWh\n", pulseCount, power, todaykWh);

  char tmp[80];
  sprintf(tmp, "%d\t%.4f W\t%.3f kWh ms: %lu\n", pulseCount, power, todaykWh, millis());
  publishMQTT(DebugTopic, tmp);

  lastPulseTime = currentTime;
  tSensorOneLed.enable();
  interrupts();
}

void sendTelemetry() {
  const size_t bufferSize = JSON_OBJECT_SIZE(1) + JSON_OBJECT_SIZE(5);
  DynamicJsonBuffer respBuffer(bufferSize);
  JsonObject& json = respBuffer.createObject();

  time_t t = (long) timeClient.getEpochTime();

  char buf[80];
  sprintf(buf, "%d-%d-%dT%s", year(t), month(t), day(t), timeClient.getFormattedTime().c_str());
  json["Time"] = buf;
  
  JsonObject& energy = json.createNestedObject("ENERGY");
  energy["Power"] = power;
  energy["Today"] = todaykWh;
  energy["Total"] = totalkWh;
  energy["Yesterday"] = yesterdaykWh;

  String tele = "";
  json.printTo(tele);
  publishMQTT(TelemetryTopic, tele);
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
  // resetEEPROM();
  // readFromEEPROM();


  setupWifiManager();
  setupOTA();
  setupMQTT();
  timeClient.begin();

  tCheckWifiStatus.enable();
  // tSaveToEEPROM.enable();
  tUpdateTime.enable();
  tSendTelemetry.enable();

  // lastly enable interrupts
  pinMode(SENSOR_ONE_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(SENSOR_ONE_PIN), sensorOneHandler, RISING);
  Log("\nEnergy meter setup done\n\n");
}

void loop() {
  if(OTA_ON) {
    noInterrupts();
    ArduinoOTA.handle();
    interrupts();
  }

  if(!mqttclient.connected()) {
    connectMQTT();
  } {
    mqttclient.loop();
  }

  taskManager.execute();
  
  // see if next day
  currentDay = timeClient.getDay();
  if(currentDay != lastDay) {
    // a new day
    lastDay = currentDay;
    yesterdaykWh = todaykWh;
    totalkWh += todaykWh;
    todaykWh = 0;
    pulseCount = 0;
    saveToEEPROM();
  } 
}