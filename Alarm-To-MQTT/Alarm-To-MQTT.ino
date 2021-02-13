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

// LED
#define LED_BUILTIN                       2

// ALARM
#define BUZZER_PIN                        27
#define RED_LED_PIN                       33
#define GREEN_LED_PIN                     25
#define SS_PIN                            21
#define RST_PIN                           22
#define IRQ_PIN                           15

////////////////////////////////
// Variables
////////////////////////////////
const int numALARMs =                     2;
char hostname[32], buffer[64];
char telemetryTopic[96], commandTopic[64], commandTopicRestart[64], commandTopicStart[64], commandTopicStop[64], commandTopicAP_Start[64], commandTopicAP_Stop[64], debugTopic[64];
// Thing
bool Active = true;
const String ThingId = CONFIG_THING_ID;
// RFID
byte readCard[10];
uint8_t cardSize;
volatile bool bNewInt = false;
byte regVal = 0x7F;
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

/*
 * The function sending to the MFRC522 the needed commands to activate the reception
 */
void activateRec(MFRC522 mfrc522) {
  mfrc522.PCD_WriteRegister(mfrc522.FIFODataReg, mfrc522.PICC_CMD_REQA);
  mfrc522.PCD_WriteRegister(mfrc522.CommandReg, mfrc522.PCD_Transceive);
  mfrc522.PCD_WriteRegister(mfrc522.BitFramingReg, 0x87);
}

/*
 * The function to clear the pending interrupt bits after interrupt serving routine
 */
void clearInt(MFRC522 mfrc522) {
  mfrc522.PCD_WriteRegister(mfrc522.ComIrqReg, 0x7F);
}

////////////////////////////////
// Interrupts
////////////////////////////////
/**
 * MFRC522 interrupt serving routine
 */
void IRAM_ATTR MFRC522_ISR() {
  bNewInt = true;
}

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
        if (millis() - CHANGE_MILLIS > 500) {
          OPEN = _OPEN;
          CLOSED = !_OPEN;

          MQTTclient.publish(telemetryTopic, OPEN ? "Door Open!" : "Door Closed!");
          
          _CHANGING = false;
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

  setupMFRC522();

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

  if (bNewInt) {
    getID();
  }

  activateRec(mfrc522);
  delay(100);
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
  MQTTclient.setServer(CONFIG_MQTT_SERVER, CONFIG_MQTT_SERVERPORT);
  MQTTclient.setCallback(MQTTOnMessage);

  MQTTclient.setBufferSize(2048);

  String TopicString = "telemetry/" + ThingId + "/alarm";
  TopicString.toCharArray(telemetryTopic, 64);

  TopicString = "command/" + ThingId + "/+";
  TopicString.toCharArray(commandTopic, 64);

  TopicString = "command/" + ThingId + "/restart";
  TopicString.toCharArray(commandTopicRestart, 64);

  TopicString = "command/" + ThingId + "/start";
  TopicString.toCharArray(commandTopicStart, 64);

  TopicString = "command/" + ThingId + "/stop";
  TopicString.toCharArray(commandTopicStop, 64);

  TopicString = "command/" + ThingId + "/ap_start";
  TopicString.toCharArray(commandTopicAP_Start, 64);

  TopicString = "command/" + ThingId + "/ap_stop";
  TopicString.toCharArray(commandTopicAP_Stop, 64);

  TopicString = "debug/" + ThingId;
  TopicString.toCharArray(debugTopic, 64);
  
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
    if (MQTTclient.connect(hostname, CONFIG_MQTT_USERNAME, CONFIG_MQTT_PASSWORD)) {
        
      MQTTclient.subscribe(commandTopic);

      MQTTclient.publish(debugTopic, "Alarm Connected!");
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
  if (str2int(topic) == str2int(commandTopicRestart)) {
    MQTTclient.publish(debugTopic, "Restarting Alarm!");
    delay(1000);
    ESP.restart();
  }

  if (str2int(topic) == str2int(commandTopicStart)) {
    MQTTclient.publish(debugTopic, "Starting Alarm!");
    digitalWrite(RED_LED_PIN, HIGH);
    digitalWrite(GREEN_LED_PIN, HIGH);
    Active = true;
    return;
  }

  if (str2int(topic) == str2int(commandTopicStop)) {
    MQTTclient.publish(debugTopic, "Stoping Alarm!");
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(GREEN_LED_PIN, LOW);
    Buzzer(false);
    Active = false;
    return;
  }

  if (str2int(topic) == str2int(commandTopicAP_Start)) {
    MQTTclient.publish(debugTopic, "Starting AP!");
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(hostname);
    return;
  }

  if (str2int(topic) == str2int(commandTopicAP_Stop)) {
    MQTTclient.publish(debugTopic, "Stoping AP!");
    WiFi.mode(WIFI_STA);
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
  ArduinoOTA.setPassword((const char *)CONFIG_OTA_PASSWORD);

  // Set OTA Password
  ArduinoOTA.setPort(8266);

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
  detachInterrupt(digitalPinToInterrupt(IRQ_PIN));
  
  if (!mfrc522.PICC_ReadCardSerial()) {
    setupMFRC522();
    return false;
  }

  cardSize = mfrc522.uid.size;
  
  if (cardSize > 0) {
    for (uint8_t i = 0; i < cardSize; i++) {
      readCard[i] = mfrc522.uid.uidByte[i];
    }

    switch (cardSize) {
      case 4:
        snprintf(buffer, 64, "{\"id\": \"0x%02X%02X%02X%02X\", \"size\": %d}", readCard[0], readCard[1], readCard[2], readCard[3], cardSize);
        break;
      case 7:
        snprintf(buffer, 64, "{\"id\": \"0x%02X%02X%02X%02X%02X%02X%02X\", \"size\": %d}", readCard[0], readCard[1], readCard[2], readCard[3], readCard[4], readCard[5], readCard[6], cardSize);
        break;
      default:
        clearInt(mfrc522);
        bNewInt = false;
        attachInterrupt(digitalPinToInterrupt(IRQ_PIN), MFRC522_ISR, FALLING);
        return false;
    }
  } else {
    digitalWrite(RED_LED_PIN, LOW);
    delay(250);
    digitalWrite(RED_LED_PIN, HIGH);
    delay(250);
    digitalWrite(RED_LED_PIN, LOW);
    delay(250);
    digitalWrite(RED_LED_PIN, HIGH);
    delay(250);
    digitalWrite(RED_LED_PIN, LOW);
  }

  clearInt(mfrc522);
  mfrc522.PICC_HaltA();

  MQTTclient.publish(telemetryTopic, buffer);

  if (ARMED) {
    ARMED = false;
  }
  
  bNewInt = false;

  attachInterrupt(digitalPinToInterrupt(IRQ_PIN), MFRC522_ISR, FALLING);
  
  return true;
}

void Buzzer(boolean status) {
  digitalWrite(BUZZER_PIN, status);
}

void setupMFRC522() {
  SPI.begin();                                                      // MFRC522 Hardware uses SPI protocol
  mfrc522.PCD_Init();                                               // Initialize MFRC522 Hardware
  
  /* read and printout the MFRC522 version (valid values 0x91 & 0x92)*/
  byte readReg = mfrc522.PCD_ReadRegister(mfrc522.VersionReg);
  snprintf(buffer, 64, "{\"version\": \"0x%02X\"}", readReg);
  MQTTclient.publish(debugTopic, buffer);

  /* setup the IRQ pin*/
  pinMode(IRQ_PIN, INPUT_PULLUP);

  regVal = 0xA0; //rx irq
  mfrc522.PCD_WriteRegister(mfrc522.ComIEnReg, regVal);

  bNewInt = false; //interrupt flag

  /*Activate the interrupt*/
  attachInterrupt(digitalPinToInterrupt(IRQ_PIN), MFRC522_ISR, FALLING);
}
