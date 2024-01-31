// Sender Address: 48:27:E2:FD:6B:A4
// Reciever MAC Address: EC:DA:3B:63:AF:EC

#include <Arduino.h>
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
  dioPin = A4;

TM1637TinyDisplay6 display(clkPin, dioPin);

//====================
// General variables

int currentTime = 0;
boolean moveCommand = false;
boolean saveCommand = false;

//====================
// Setup

void setup() {
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
  //display.setBrightness(4);
  Serial.begin(9600);
}

//====================
// Main loop

void loop() {
  currentTime = millis();
  int switchValue = digitalRead(switchPin);
  int knob1Value = 4095 - analogRead(knobPin[0]);
  int knob2Value = 4095 - analogRead(knobPin[1]);

  batteryIndicator(knob1Value * 100 / 4095);
  checkButtonX();
  checkButtonY();
  statusIndicator();
  delay(1);
}

//====================
// batteryIndicator function

void batteryIndicator(int batteryLevel) {
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
    digitalWrite(ledPin[3], LOW);
    digitalWrite(ledPin[2], LOW);
  }
}

//====================
// checkButton functions

int holdDelay = 500;
int deadBand = 205;

int startTimeX = 0;
int pressedX = -1;
boolean holdingX = false;

void checkButtonX() {
  if (pressedX == -1) {
    // Check joystick value
    int joystickXValue = 2048 - analogRead(joystickPin[0]);
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
      Serial.print("Holding button ");
      Serial.println(pressedX);
      moveCommand = true;
    }
    // If released
    else if (currentButtonX == HIGH) {
      if (!holdingX) {
        Serial.print("Released button ");
        Serial.println(pressedX);
        saveCommand = true;
      }
      pressedX = -1;
    }
  }
}

int startTimeY = 0;
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
      Serial.print("Holding button ");
      Serial.println(pressedY);
      moveCommand = true;
    }
    // If released
    else if (currentButtonY == HIGH) {
      if (!holdingY) {
        Serial.print("Released button ");
        Serial.println(pressedY);
        saveCommand = true;
      }
      pressedY = -1;
    }
  }
}

//====================
// statusIndicator function

int onDelay = 500;

int onTime = 0;

void statusIndicator() {
  if (moveCommand) {
    digitalWrite(ledPin[1], HIGH);
    digitalWrite(ledPin[0], HIGH);
    moveCommand = false;
  }
  else if (saveCommand) {
    digitalWrite(ledPin[1], HIGH);
    digitalWrite(ledPin[0], LOW);
    if (currentTime - onTime > onDelay) {
      saveCommand = false;
    }
  }
  else {
    digitalWrite(ledPin[1], LOW);
    digitalWrite(ledPin[0], LOW);
    onTime = currentTime;
  }
}
