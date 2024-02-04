// Remote Controller Address: 48:27:E2:FD:6B:A4
// Onboard Controller Address: EC:DA:3B:63:AF:EC

#include <esp_now.h>
#include <WiFi.h>

//====================
// Pins

//====================
// General variables

int currentTime = 0;
int receiveTime = 0;

int dataSize;
int speedSetpointUpper;
int speedSetpointLower;

//====================
// ESP-NOW definitions

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

  //====================
  // ESP-NOW setup

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_register_recv_cb(OnDataRecv);
}

//====================
// Main loop

void loop() {
  currentTime = millis();
  
  Serial.print(dataSize);
  Serial.print(", ");
  Serial.print(speedSetpointUpper);
  Serial.print(", ");
  Serial.print(speedSetpointLower);
  Serial.println(" ");

  checkTimeout();
  delay(1);
}

//====================
// checkTimeout function

int timeoutDelay = 10;

void checkTimeout() {
  if (currentTime - receiveTime > timeoutDelay) {
    dataSize = 0;
    speedSetpointUpper = 0;
    speedSetpointLower = 0;
  }
}
