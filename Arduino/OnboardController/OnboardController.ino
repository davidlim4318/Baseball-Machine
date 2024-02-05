// Remote Controller Address: 48:27:E2:FD:6B:A4
// Onboard Controller Address: EC:DA:3B:63:AF:EC

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>

//====================
// Pins

const int
  escPin[2] = { 2, 3 },
  hallSensorPin[2] = { 11, 12 };
;

//====================
// General variables/objects

int currentTime = 0;

int speedSetpointUpper;
int speedSetpointLower;

int speedUpper;
int speedLower;

int receiveTime = 0;
int dataSize;

boolean connectionTimeout = false;

Servo escUpper;
Servo escLower;

//====================
// ESP-NOW definitions to send message

uint8_t broadcastAddress[] = { 0x48, 0x27, 0xE2, 0xFD, 0x6B, 0xA4};

typedef struct struct_message_onboard {
  int value1;
  int value2;
} struct_message_onboard;

struct_message_onboard messageToSend;

esp_now_peer_info_t peerInfo;

esp_now_send_status_t currentStatus;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (currentStatus != status) {
    currentStatus = status;
    Serial.print("Connection Status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
  }
}

//====================
// ESP-NOW definitions to receive message

typedef struct struct_message_remote {
  int value1;
  int value2;
} struct_message_remote;

struct_message_remote messageToReceive;

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&messageToReceive, incomingData, sizeof(messageToReceive));

  receiveTime = currentTime;
  dataSize = len;
  speedSetpointUpper = messageToReceive.value1;
  speedSetpointLower = messageToReceive.value2;
}

//====================
// Setup

void setup() {

  //====================
  // General setup

  Serial.begin(9600);

  pinMode(hallSensorPin[0],INPUT_PULLUP);
  pinMode(hallSensorPin[1],INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(hallSensorPin[0]), measureSpeedUpper, CHANGE);
  attachInterrupt(digitalPinToInterrupt(hallSensorPin[1]), measureSpeedLower, CHANGE);

  analogReadResolution(12);
  analogWriteResolution(12);

  //====================
  // ESC Calibration

  escUpper.attach(escPin[0]);
  escLower.attach(escPin[1]);
  escUpper.writeMicroseconds(2000);
  escLower.writeMicroseconds(2000);
  delay(3000);
  escUpper.writeMicroseconds(1000);
  escLower.writeMicroseconds(1000);
  delay(2000);

  //====================
  // ESP-NOW setup

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
}

//====================
// Main loop

void loop() {
  currentTime = millis();

  checkMeasureSpeedTimeout();

  Serial.print(speedUpper);
  Serial.print(", ");
  Serial.print(speedLower);
  Serial.print(", ");

  sendMessage();
  checkConnectionTimeout();

  Serial.print(dataSize);
  Serial.print(", ");
  Serial.print(speedSetpointUpper);
  Serial.print(", ");
  Serial.print(speedSetpointLower);
  Serial.println(" ");
  
  escUpper.writeMicroseconds(speedSetpointUpper * 1000 / 5676 + 1000);
  escLower.writeMicroseconds(speedSetpointLower * 1000 / 5676 + 1000);

  delay(1);
}

//====================
// sendMessage function

void sendMessage() {
  messageToSend.value1 = speedUpper;
  messageToSend.value2 = speedLower;

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &messageToSend, sizeof(messageToSend));
}

//====================
// measureSpeedUpper/Lower functions

int changeTime1 = 0;
int changeCounter1 = 0;

void measureSpeedUpper() {
  changeCounter1 = ++changeCounter1;
  if (changeCounter1 >= 14) {
    int delta1 = micros() - changeTime1;
    changeTime1 = micros();
    speedLower = changeCounter1 * 1000000.0 * 60.0 / delta1 / 14.0;
    changeCounter1 = 0;
  }
}

int changeTime2 = 0;
int changeCounter2 = 0;

void measureSpeedLower() {
  changeCounter2 = ++changeCounter2;
  if (changeCounter2 >= 14) {
    int delta2 = micros() - changeTime2;
    changeTime2 = micros();
    speedUpper = changeCounter2 * 1000000.0 * 60.0 / delta2 / 14.0;
    changeCounter2 = 0;
  }
}

//====================
// checkMeasureSpeedTimeout function

int measureSpeedTimeoutDelay = 1000000;

void checkMeasureSpeedTimeout() {
  if (currentTime - changeTime1 >= measureSpeedTimeoutDelay) {
    changeTime1 = micros();
    speedUpper = 0;
    changeCounter1 = 0;
  }
  if (currentTime - changeTime2 >= measureSpeedTimeoutDelay) {
    changeTime2 = micros();
    speedLower = 0;
    changeCounter2 = 0;
  }
}

//====================
// checkConnectionTimeout function

int connectionTimeoutDelay = 100;

void checkConnectionTimeout() {
  if (currentTime - receiveTime > connectionTimeoutDelay) {
    connectionTimeout = true;
    dataSize = 0;
    speedUpper = 0;
    speedLower = 0;
  }
  else {
    connectionTimeout = false;
  }
}
