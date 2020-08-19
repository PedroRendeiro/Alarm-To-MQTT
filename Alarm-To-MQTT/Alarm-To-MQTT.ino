#include <WiFi.h>
#include <WiFiClient.h>
#include <ArduinoOTA.h>                   // For OTA Programming
#include <ArduinoJson.h>                  // JSON
#include <PubSubClient.h>                 // MQTT
#include <SPI.h>                          // RC522 Module uses SPI protocol
#include <MFRC522.h>                      // Library for Mifare RC522 Devices
#include "secrets.h"

// WiFi
const char* ssid =                        CONFIG_WIFI_SSID;
const char* password =                    CONFIG_WIFI_PASSWORD;

// OTA
#define OTA_PASSWORD                      CONFIG_OTA_PASSWORD

// MQTT
#define MQTT_SERVER                       CONFIG_MQTT_SERVER
#define MQTT_SERVERPORT                   CONFIG_MQTT_SERVERPORT
#define MQTT_USERNAME                     CONFIG_MQTT_USER
#define MQTT_KEY                          CONFIG_MQTT_PASSWORD

// LED
#define LED_BUILTIN                       2

// ALARM
#define BUZZER_PIN                        27
#define RED_LED_PIN                       33
#define GREEN_LED_PIN                     25
#define SS_PIN                            21
#define RST_PIN                           22

////////////////////////////////
// Variables
////////////////////////////////
const int numALARMs =                     2;
char hostname[32], MQTT_TOPIC[96], buffer[512];
// Thing
bool Active = true;
const String ThingId = CONFIG_THING_ID;
// RFID
byte readCard[10];
uint8_t cardSize;
// Alarm
bool OPEN, CLOSED, ARMED = false;
// CLOSED -> All closed
// OPEN -> All open
// !CLOSED -> At least one open
// !OPEN -> At least one closed

////////////////////////////////
// Complex Variables
////////////////////////////////
WiFiClient TCPclient;                        // TCP Client
PubSubClient MQTTclient(TCPclient);          // MQTT Client
MFRC522 mfrc522(SS_PIN, RST_PIN);

////////////////////////////////
// Util functions
////////////////////////////////
constexpr unsigned int str2int(const char* str, int h = 0) {
    return !str[h] ? 5381 : (str2int(str, h+1) * 33) ^ str[h];
}

#define REPORTING_PERIOD                  30000
#define WATCHDOG_TIMEOUT_PERIOD           86400000

////////////////////////////////
// Structs
////////////////////////////////
struct ALARM {
  private:
    bool _OPEN = true, _CHANGING = false;
    uint64_t CHANGE_MILLIS = millis();
  public:
    uint8_t SENSOR_PIN;
    bool OPEN = true, CLOSED = true;
    
    ALARM(uint8_t _SENSOR_PIN) {
      SENSOR_PIN = _SENSOR_PIN;
    }

    void begin() {
      pinMode(SENSOR_PIN, INPUT_PULLDOWN);
    }

    void loop() {
      _OPEN = !digitalRead(SENSOR_PIN);

      if (_CHANGING) {
        if (millis() - CHANGE_MILLIS > 3000) {
          OPEN = _OPEN;
          CLOSED = !_OPEN;
          
          _CHANGING = false;
          
          MQTTclient.publish("alarm", OPEN ? "OPEN" : "CLOSED");
        }
        if (_OPEN == OPEN) {
          _CHANGING = false;
        }
      } else {
        if (_OPEN != OPEN) {
          CHANGE_MILLIS = millis();
          _CHANGING = true;
        }
      }
    }
};
ALARM ALARMs[numALARMs] = {ALARM(26), ALARM(32)};

////////////////////////////////
// Setup
////////////////////////////////
void setup() { 
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, LOW);
  pinMode(RED_LED_PIN, OUTPUT); digitalWrite(RED_LED_PIN, HIGH);
  pinMode(GREEN_LED_PIN, OUTPUT); digitalWrite(GREEN_LED_PIN, HIGH);

  setupWiFi();
  setupOTA();
  setupMQTT();

  SPI.begin();                                                      // MFRC522 Hardware uses SPI protocol
  mfrc522.PCD_Init();                                               // Initialize MFRC522 Hardware
  mfrc522.PCD_SetAntennaGain(mfrc522.RxGain_max);

  for (int Alarm = 0; Alarm < numALARMs; Alarm++) {
    ALARMs[Alarm].begin();
  }

  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(LED_BUILTIN, LOW);
}

////////////////////////////////
// Loop
////////////////////////////////
void loop() {
  connectWiFi();
  loopMQTT();
  loopOTA();

  if (!Active) {
    return;
  }
  
  for (int Alarm = 0; Alarm < numALARMs; Alarm++) {
    ALARMs[Alarm].loop();
  }

  OPEN = true; CLOSED = true;
  for (int Alarm = 0; Alarm < numALARMs; Alarm++) {
    OPEN &= ALARMs[Alarm].OPEN;
    CLOSED &= ALARMs[Alarm].CLOSED;
  }

  if (ARMED) {
    if (!CLOSED) {
      Buzzer(true);
    }

    if (getID()) {
      ARMED = false;
    }
    
    digitalWrite(RED_LED_PIN, HIGH);
    digitalWrite(GREEN_LED_PIN, LOW);
  } else {
    Buzzer(false);
    
    if (CLOSED) {
      ARMED = true;
    }

    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(GREEN_LED_PIN, HIGH);
  }
}

////////////////////////////////
// WiFi
////////////////////////////////
void setupWiFi() {
  uint32_t id = 0;
  for(int i=0; i<17; i=i+8) {
    id |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }
  
  String hostnameString = "ALARM-" + String(id);
  hostnameString.toCharArray(hostname, 32);

  WiFi.mode(WIFI_STA);

  // Set your Static IP address
  IPAddress local_IP(192, 168, 1, 4);
  // Set your Gateway IP address
  IPAddress gateway(192, 168, 1, 254);
  
  IPAddress subnet(255, 255, 255, 0);
  IPAddress primaryDNS(192, 168, 1, 254);
  IPAddress secondaryDNS(8, 8, 8, 8);

  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    ESP.restart();
  }

  WiFi.setHostname(hostname);

  connectWiFi();
}

bool connectWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ssid, password);

    const int kRetryCountWiFi = 20;
    int retryCountWiFi = 0;
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      if (retryCountWiFi++ > kRetryCountWiFi) {
        ESP.restart();
      }
    }
  }
  
  return WiFi.status() == WL_CONNECTED;
}

////////////////////////////////
// MQTT
////////////////////////////////
void setupMQTT() {
  MQTTclient.setServer(MQTT_SERVER, MQTT_SERVERPORT);
  MQTTclient.setCallback(MQTTOnMessage);

  MQTTclient.setBufferSize(2048);

  String commandTopicString = "telemetry/" + ThingId + "/alarm";
  commandTopicString.toCharArray(MQTT_TOPIC, 96);
  
  loopMQTT();
}

void loopMQTT() {
  if (!MQTTclient.connected()) {
    reconnectMQTT();
  }
  MQTTclient.loop();
}

void reconnectMQTT() {
  const int kRetryCountMQTT = 40;
  int retryCountMQTT = 0;
  // Loop until we're reconnected
  while (!MQTTclient.connected()) {    
    // Attempt to connect
    if (MQTTclient.connect(hostname, MQTT_USERNAME, MQTT_KEY)) {

      char commandTopicChar[64], debugTopicChar[64];
      String commandTopicString, debugTopicString;

      commandTopicString = "command/" + ThingId + "/+";
      commandTopicString.toCharArray(commandTopicChar, 64);
      debugTopicString = "debug/" + ThingId;
      debugTopicString.toCharArray(debugTopicChar, 64);
        
      MQTTclient.subscribe(commandTopicChar);

      MQTTclient.publish(debugTopicChar, "Alarm Connected!");
    } else {
      // Wait 5 seconds before retrying
      unsigned long now = millis();
      while (millis() - now < 5000) {
        loopOTA();
        yield();
      }
    }
    if (retryCountMQTT++ > kRetryCountMQTT) {
      ESP.restart();
    }
  }
}

void MQTTOnMessage(char* topic, byte* payload, unsigned int length) {
  char commandTopicChar[64], debugTopicChar[64];
  String commandTopicString, debugTopicString;

  debugTopicString = "debug/" + ThingId;
  debugTopicString.toCharArray(debugTopicChar, 64);

  commandTopicString = "command/" + ThingId + "/restart";
  commandTopicString.toCharArray(commandTopicChar, 64);
  if (str2int(topic) == str2int(commandTopicChar)) {
    MQTTclient.publish(debugTopicChar, "Restarting Alarm!");
    ESP.restart();
  }

  commandTopicString = "command/" + ThingId + "/start";
  commandTopicString.toCharArray(commandTopicChar, 64);
  if (str2int(topic) == str2int(commandTopicChar)) {
    MQTTclient.publish(debugTopicChar, "Starting Alarm!");
    digitalWrite(RED_LED_PIN, HIGH);
    digitalWrite(GREEN_LED_PIN, HIGH);
    Active = true;
    return;
  }

  commandTopicString = "command/" + ThingId + "/stop";
  commandTopicString.toCharArray(commandTopicChar, 64);
  if (str2int(topic) == str2int(commandTopicChar)) {
    MQTTclient.publish(debugTopicChar, "Stoping Alarm!");
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(GREEN_LED_PIN, LOW);
    Buzzer(false);
    Active = false;
    return;
  }

  String topicString(topic);
  topicString += "/reply";
  char topicChar[64];
  topicString.toCharArray(topicChar, 64);
  MQTTclient.publish(topicChar, "Invalid command received!");
}

////////////////////////////////
// OTA
////////////////////////////////
void setupOTA() {
  // Set OTA Hostname
  ArduinoOTA.setHostname(hostname);

  // Set OTA Password
  ArduinoOTA.setPassword((const char *)OTA_PASSWORD);

  // Init OTA
  ArduinoOTA.begin();

  // Loop OTA
  loopOTA();
}

void loopOTA() {
  ArduinoOTA.handle();
}

////////////////////////////////
// Alarm
////////////////////////////////
boolean getID() {
  mfrc522.PCD_AntennaOn();
  
  // Getting ready for Reading PICCs
  if (!mfrc522.PICC_IsNewCardPresent()) { //If a new PICC placed to RFID reader continue
    return false;
  }
  if (!mfrc522.PICC_ReadCardSerial()) {   //Since a PICC placed get Serial and continue
    return false;
  }
  
  cardSize = mfrc522.uid.size;
  for (uint8_t i = 0; i < cardSize; i++) {
    readCard[i] = mfrc522.uid.uidByte[i];
  }
  mfrc522.PICC_HaltA(); // Stop reading

  switch (cardSize) {
    case 4:
      snprintf(buffer, 512, "{\"id\": \"%02X%02X%02X%02X\"}", readCard[0], readCard[1], readCard[2], readCard[3]);
      break;
    case 7:
      snprintf(buffer, 512, "{\"id\": \"%02X%02X%02X%02X%02X%02X%02X\"}", readCard[0], readCard[1], readCard[2], readCard[3], readCard[4], readCard[5], readCard[6]);
      break;
    default:
      break;
  }

  MQTTclient.publish("alarm/scanned", buffer);

  mfrc522.PCD_AntennaOff();
  
  return true;
}

void Buzzer(boolean status) {
  digitalWrite(BUZZER_PIN, status);
}
