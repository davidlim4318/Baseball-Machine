// Remote Controller Address: 48:27:E2:FD:6B:A4
// Onboard Controller Address: EC:DA:3B:63:AF:EC

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

boolean errorCondition = false;
boolean moveCommand = false;
boolean saveCommand = false;

int speedSetpointUpper;
int speedSetpointLower;

TM1637TinyDisplay6 display(clkPin, dioPin);

//====================
// ESP-NOW definitions

uint8_t broadcastAddress[] = { 0xEC, 0xDA, 0x3B, 0x63, 0xAF, 0xEC };

typedef struct struct_message_remote {
  int value1;
  int value2;
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
// Display Data

const uint8_t ANIMATION[12][6] = {
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  // Frame 0
  { 0x00, 0x00, 0x80, 0x00, 0x00, 0x00 },  // Frame 1
  { 0x00, 0x80, 0x80, 0x00, 0x00, 0x00 },  // Frame 2
  { 0x80, 0x80, 0x80, 0x00, 0x00, 0x00 },  // Frame 3
  { 0x80, 0x80, 0x80, 0x00, 0x00, 0x80 },  // Frame 4
  { 0x80, 0x80, 0x80, 0x00, 0x80, 0x80 },  // Frame 5
  { 0x80, 0x80, 0x80, 0x80, 0x80, 0x80 },  // Frame 6
  { 0x80, 0x80, 0x00, 0x80, 0x80, 0x80 },  // Frame 7
  { 0x80, 0x00, 0x00, 0x80, 0x80, 0x80 },  // Frame 8
  { 0x00, 0x00, 0x00, 0x80, 0x80, 0x80 },  // Frame 9
  { 0x00, 0x00, 0x00, 0x80, 0x80, 0x00 },  // Frame 10
  { 0x00, 0x00, 0x00, 0x80, 0x00, 0x00 }   // Frame 11
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
}

//====================
// Main loop

void loop() {
  currentTime = millis();
  
  int knob1Value = 4095 - analogRead(knobPin[0]);
  int knob2Value = 4095 - analogRead(knobPin[1]);
  speedSetpointUpper = knob1Value;
  speedSetpointLower = knob2Value;

  checkButtonX();
  checkButtonY();
  checkFeedButton();
  sendMessage();

  indicateStatus();
  //playAnimation();
  indicateBattery(knob1Value * 100 / 4095);
  delay(1);
}

//====================
// checkButtonX/Y functions

int holdDelay = 500;
int debounceDelay = 300;
int deadBand = 205;

int startTimeX = 0;
int holdTimeX = 0;
int pressedX = -1;
boolean holdingX = false;

void checkButtonX() {
  if (pressedX == -1) {
    // Check joystick value
    int switchValue = 1 - 2*digitalRead(switchPin);
    int joystickXValue = switchValue*(2048 - analogRead(joystickPin[0]));
    if (abs(joystickXValue) > deadBand) {
      Serial.print("Holding joystick X at ");
      Serial.println(joystickXValue);
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
boolean holdingY = false;

void checkButtonY() {
  if (pressedY == -1) {
    // Check joystick value
    int joystickYValue = analogRead(joystickPin[1]) - 2048;
    if (abs(joystickYValue) > deadBand) {
      Serial.print("Holding joystick Y at ");
      Serial.println(joystickYValue);
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
  int feedForward = digitalRead(buttonPin[6]);
  if (feedForward == LOW) {
    Serial.println("Feeding forward");
    moveCommand = true;
  }
  int feedReverse = digitalRead(buttonPin[7]);
  if (feedReverse == LOW) {
    Serial.println("Feeding reverse");
    moveCommand = true;
  }
}

//====================
// sendMessage function

void sendMessage() {
  messageToSend.value1 = speedSetpointUpper;
  messageToSend.value2 = speedSetpointLower;

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &messageToSend, sizeof(messageToSend));
}

//====================
// indicateStatus function

int onDelay = 500;
int saveTime = 0;

void indicateStatus() {
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
// playAnimation function

int frameDelay = 200;
int frameTime = 0;
int counter = 0;

void playAnimation() {
  if (currentTime - frameTime > frameDelay) {
    display.setSegments(ANIMATION[counter]);
    frameTime = currentTime;
    counter++;
    if (counter > 11) {
      counter = 0;
    }
  }
}

//====================
// indicateBattery function

void indicateBattery(int batteryLevel) {
  batteryLevel = constrain(batteryLevel,0,99);
  if (batteryLevel >= 66) {
    digitalWrite(ledPin[4], HIGH);
    digitalWrite(ledPin[3], HIGH);
    analogWrite(ledPin[2], 4095 * (batteryLevel - 66) / 33);
  }
  else if (batteryLevel >= 33) {
    digitalWrite(ledPin[4], HIGH);
    analogWrite(ledPin[3], 4095 * (batteryLevel - 33) / 33);
    digitalWrite(ledPin[2], LOW);
  }
  else {
    analogWrite(ledPin[4], 4095 * batteryLevel / 33);
    if (batteryLevel == 0) {
      digitalWrite(ledPin[4], LOW);
    }
    digitalWrite(ledPin[3], LOW);
    digitalWrite(ledPin[2], LOW);
  }
}
