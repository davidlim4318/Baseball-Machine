#include <Arduino.h>
#include <ESP32Servo.h>

//====================
// Pins

const int
  escPin[2] = { 2, 4 };

//====================
// General variables/objects

int currentTime = 0;

Servo escUpper;
Servo escLower;

//====================
// Setup

void setup() {

  //====================
  // General setup

  Serial.begin(9600);

  analogReadResolution(12);
  analogWriteResolution(12);

  //====================
  // ESC Setup

  escUpper.attach(escPin[0]);
  escLower.attach(escPin[1]);

  delay(1000);

  Serial.println("BEGIN CALIBRATION.");
  delay(1000);

  // escUpper.writeMicroseconds(1500);
  // escLower.writeMicroseconds(1500);
  // Serial.println("SET NEUTRAL POSITION ...");
  // delay(3000);

  escUpper.writeMicroseconds(2000);
  escLower.writeMicroseconds(2000);
  Serial.println("SET FORWARD POSITION ...");
  delay(3000);

  escUpper.writeMicroseconds(1000);
  escLower.writeMicroseconds(1000);
  Serial.println("SET BACKWARD POSITION ...");
  delay(3000);

  // escUpper.writeMicroseconds(1500);
  // escLower.writeMicroseconds(1500);
  Serial.println("CALIBRATION COMPLETE.");
  delay(1000);
}

//====================
// Main loop

void loop() {
  int value = analogRead(A0);
  escUpper.writeMicroseconds(value * 1000 / 4095 + 1000);
  escLower.writeMicroseconds(value * 1000 / 4095 + 1000);
  Serial.println(value);
  delay(10);
}

