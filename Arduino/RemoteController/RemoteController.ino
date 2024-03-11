// Remote Controller Address: 48:27:E2:FD:6B:A4
// Onboard Controller Address: EC:DA:3B:60:D6:18

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <TM1637TinyDisplay6.h>

//====================
// Pins

const int
  ledPin[5] = { 2, 3, 4, 5, 6 },
  switchPin = 7,
  buttonPin[8] = { 8, 9, 10, 11, 12, 13, A6, A3 },
  joystickPin[2] = { A1, A0 },
  knobPin[2] = { A7, A2 },
  clkPin = A5,
  dioPin = A4
;

//====================
// General variables/objects

int currentTime = 0;

bool errorCondition = false;
bool moveCommand = false;
bool saveCommand = false;

int speedSetpointUpper;
int speedSetpointLower;

int moveX;
int moveY;
bool moveAutoX;
bool moveAutoY;

int feed;

int speedUpper;
int speedLower;

int batterySOC;

int receiveTime = 0;
int dataSize;

bool connectionTimeout = false;
bool knobReset = false;

TM1637TinyDisplay6 display(clkPin, dioPin);

//====================
// ESP-NOW definitions to send mesage

uint8_t broadcastAddress[] = { 0xEC, 0xDA, 0x3B, 0x60, 0xD6, 0x18 };

typedef struct struct_message_remote {
  int value1;
  int value2;
  int value3;
  int value4;
  bool value5;
  bool value6;
  int value7;
} struct_message_remote;

struct_message_remote messageToSend;

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

typedef struct struct_message_onboard {
  int value1;
  int value2;
  int value3;
  int value4;
  int value5;
} struct_message_onboard;

struct_message_onboard messageToReceive;

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&messageToReceive, incomingData, sizeof(messageToReceive));

  receiveTime = currentTime;
  dataSize = len;
  speedUpper = messageToReceive.value1;
  speedLower = messageToReceive.value2;
  batterySOC = messageToReceive.value5;
}

//====================
// Animation Data

const uint8_t ANIMATION1[6][6] = {
  { 0x00, 0x00, 0x80, 0x00, 0x00, 0x00 },  // Frame 0
  { 0x00, 0x80, 0x00, 0x00, 0x00, 0x00 },  // Frame 1
  { 0x80, 0x00, 0x00, 0x00, 0x00, 0x00 },  // Frame 2
  { 0x00, 0x00, 0x00, 0x80, 0x00, 0x00 },  // Frame 3
  { 0x00, 0x00, 0x00, 0x00, 0x80, 0x00 },  // Frame 4
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x80 }   // Frame 5
};

const uint8_t ANIMATION2[6][6] = {
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  // Frame 0
  { 0x00, 0x00, 0x40, 0x00, 0x00, 0x40 },  // Frame 1
  { 0x00, 0x40, 0x40, 0x00, 0x40, 0x40 },  // Frame 2
  { 0x40, 0x40, 0x40, 0x40, 0x40, 0x40 },  // Frame 3
  { 0x40, 0x40, 0x00, 0x40, 0x40, 0x00 },  // Frame 4
  { 0x40, 0x00, 0x00, 0x40, 0x00, 0x00 }   // Frame 5
};

//====================
// Setup

void setup() {

  //====================
  // General setup

  Serial.begin(9600);

  for (byte i = 0; i < 5; i++) {
    pinMode(ledPin[i], OUTPUT);
  }
  pinMode(switchPin,INPUT_PULLUP);
  for (byte i = 0; i < 8; i++) {
    pinMode(buttonPin[i], INPUT_PULLUP);
  }
  
  analogReadResolution(12);
  analogWriteResolution(12);

  display.begin();
  display.setBrightness(BRIGHT_HIGH);

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

  if (knobReset) {
    setSpeed();
  }
  else {
    checkKnobReset();
  }

  checkButtonX();
  checkButtonY();
  checkFeedButton();
  displayStatus();

  sendMessage();
  checkConnectionTimeout();

  if (connectionTimeout) {
    displayAnimation1();
  }
  else if (!knobReset) {
    displayAnimation2();
  }
  else {
    displaySpeed();
  }

  displayBattery(batterySOC);
  
  delay(10);
}

//====================
// setSpeed function

void setSpeed() {
  int knob1Value = (4095 - analogRead(knobPin[0])) * 5676 / 4095;
  int knob2Value = (4095 - analogRead(knobPin[1])) * 5676 / 4095;
  speedSetpointUpper = knob1Value;
  speedSetpointLower = knob2Value;
}

//====================
// checkKnobReset function

void checkKnobReset() {
  if (4095 - analogRead(knobPin[0]) == 0 && 4095 - analogRead(knobPin[1]) == 0) {
    knobReset = true;
  }
}

//====================
// checkButtonX/Y functions

int holdDelay = 500;
int debounceDelay = 300;
int deadBand = 205;

int startTimeX = 0;
int holdTimeX = 0;
int pressedX = -1;
bool holdingX = false;

void checkButtonX() {
  if (pressedX == -1) {
    // Check joystick value
    int switchValue = 1 - 2*digitalRead(switchPin);
    int joystickXValue = switchValue*(2048 - analogRead(joystickPin[0]));
    if (abs(joystickXValue) > deadBand) {
      Serial.print("Holding joystick X at ");
      Serial.println(joystickXValue);
      moveX = joystickXValue;
      moveCommand = true;
    }
    else {
      // Check for pressed button
      for (byte i = 0; i < 3; i++) {
        if (digitalRead(buttonPin[i]) == LOW) {
          startTimeX = currentTime;
          pressedX = i;
          holdingX = false;
          break;
        }
      }
    }
  }
  else {
    int currentButtonX = digitalRead(buttonPin[pressedX]);
    // If holding
    if (currentButtonX == LOW && currentTime - startTimeX > holdDelay) {
      holdingX = true;
      holdTimeX = currentTime;
      Serial.print("Holding button ");
      Serial.println(pressedX);
      moveCommand = true;
    }
    // If released
    else if (currentButtonX == HIGH) {
      if (!holdingX && currentTime - holdTimeX > debounceDelay) {
        Serial.print("Released button ");
        Serial.println(pressedX);
        saveCommand = true;
      }
      pressedX = -1;
    }
  }
}

int startTimeY = 0;
int holdTimeY = 0;
int pressedY = -1;
bool holdingY = false;

void checkButtonY() {
  if (pressedY == -1) {
    // Check joystick value
    int joystickYValue = analogRead(joystickPin[1]) - 2048;
    if (abs(joystickYValue) > deadBand) {
      Serial.print("Holding joystick Y at ");
      Serial.println(joystickYValue);
      moveY = joystickYValue;
      moveCommand = true;
    }
    else {
      // Check for pressed button
      for (byte i = 3; i < 6; i++) {
        if (digitalRead(buttonPin[i]) == LOW) {
          startTimeY = currentTime;
          pressedY = i;
          holdingY = false;
          break;
        }
      }
    }
  }
  else {
    int currentButtonY = digitalRead(buttonPin[pressedY]);
    // If holding
    if (currentButtonY == LOW && currentTime - startTimeY > holdDelay) {
      holdingY = true;
      holdTimeY = currentTime;
      Serial.print("Holding button ");
      Serial.println(pressedY);
      moveCommand = true;
    }
    // If released
    else if (currentButtonY == HIGH) {
      if (!holdingY && currentTime - holdTimeY > debounceDelay) {
        Serial.print("Released button ");
        Serial.println(pressedY);
        saveCommand = true;
      }
      pressedY = -1;
    }
  }
}

//====================
// checkFeedButton function

int enableDelay = 1000;

void checkFeedButton() {
  feed = 0;
  int feedReverse = digitalRead(buttonPin[7]);
  int feedForward = digitalRead(buttonPin[6]);
  if (feedReverse == LOW) {
    Serial.println("Feeding reverse");
    feed = -1;
    moveCommand = true;
  }
  else if (feedForward == LOW) {
    Serial.println("Feeding forward");
    feed = 1;
    moveCommand = true;
  }
}

//====================
// displayStatus function

int onDelay = 500;
int saveTime = 0;

void displayStatus() {
  if (saveCommand) {
    saveTime = currentTime;
    saveCommand = false;
  }
  if (errorCondition) {
    digitalWrite(ledPin[1], LOW); 
    digitalWrite(ledPin[0], HIGH);
  }
  else if (moveCommand) {
    digitalWrite(ledPin[1], HIGH);
    digitalWrite(ledPin[0], HIGH);
    moveCommand = false;
  }
  else if (currentTime - saveTime < onDelay) {
    digitalWrite(ledPin[1], HIGH);
    digitalWrite(ledPin[0], LOW);
  }
  else {
    digitalWrite(ledPin[1], LOW);
    digitalWrite(ledPin[0], LOW);
  }
}

//====================
// sendMessage function

void sendMessage() {
  messageToSend.value1 = speedSetpointUpper;
  messageToSend.value2 = speedSetpointLower;
  messageToSend.value3 = moveX;
  messageToSend.value4 = moveY;
  messageToSend.value5 = moveAutoX;
  messageToSend.value6 = moveAutoY;
  messageToSend.value7 = feed;

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &messageToSend, sizeof(messageToSend));
}

//====================
// checkConnectionTimeout function

int connectionTimeoutDelay = 100;

void checkConnectionTimeout() {
  if (currentTime - receiveTime > connectionTimeoutDelay) {
    connectionTimeout = true;
    knobReset = false;
    dataSize = 0;
    speedUpper = 0;
    speedLower = 0;
    batterySOC = 0;
  }
  else {
    connectionTimeout = false;
  }
}

//====================
// displayAnimation1/2 functions

int frameDelay = 333;
int frameTime = 0;
int counter = 0;

void displayAnimation1() {
  if (counter > 5) {
    counter = 0;
  }
  if (currentTime - frameTime > frameDelay) {
    display.setSegments(ANIMATION1[counter]);
    frameTime = currentTime;
    counter++;
  }
}

void displayAnimation2() {
  if (counter > 5) {
   counter = 0;
  }
  if (currentTime - frameTime > frameDelay) {
    display.setSegments(ANIMATION2[counter]);
    frameTime = currentTime;
    counter++;
  }
}

//====================
// displaySpeed function

void displaySpeed() {

  int speedUpperMph = speedUpper;
  int speedLowerMph = speedLower;

  uint8_t data[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

  data[0] = display.encodeDigit(speedUpperMph % 10);
  if (speedUpperMph >= 10) {
    data[1] = display.encodeDigit((speedUpperMph / 10) % 10);
    if (speedUpperMph >= 100) {
      data[2] = display.encodeDigit((speedUpperMph / 100) % 10);
    }
  }
  data[3] = display.encodeDigit(speedLowerMph % 10);
  if (speedLowerMph >= 10) {
    data[4] = display.encodeDigit((speedLowerMph / 10) % 10);
    if (speedLowerMph >= 100) {
      data[5] = display.encodeDigit((speedLowerMph / 100) % 10);
    }
  }

  display.setSegments(data);
}

//====================
// displayBattery function

void displayBattery(int batteryLevel) {
  if (batteryLevel >= 66) {
    digitalWrite(ledPin[4], HIGH);
    digitalWrite(ledPin[3], HIGH);
    digitalWrite(ledPin[2],HIGH);
  }
  else if (batteryLevel >= 33) {
    digitalWrite(ledPin[4], HIGH);
    digitalWrite(ledPin[3], HIGH);
    digitalWrite(ledPin[2], LOW);
  }
  else if (batteryLevel > 0) {
    digitalWrite(ledPin[4], HIGH);
    digitalWrite(ledPin[3], LOW);
    digitalWrite(ledPin[2], LOW);
  }
  else {
    digitalWrite(ledPin[4], LOW);
    digitalWrite(ledPin[3], LOW);
    digitalWrite(ledPin[2], LOW);
  }
}
