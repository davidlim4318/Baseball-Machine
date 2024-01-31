int buttonPin[3] = { 11, 12, 13};

void setup() {
  Serial.begin(9600);
  for (byte i = 0; i < 3; i++) {
    pinMode(buttonPin[i], INPUT_PULLUP);
  }
}

void loop() {
  checkButton();
  delay(1);
}

int holdDelay = 500;

int timeStart = 0;
int pressed = -1;
boolean holding = false;

void checkButton() {
  if (pressed == -1) {
    // Check for pressed button
    for (byte i = 0; i < 3; i ++) {
      if (digitalRead(buttonPin[i]) == LOW) {
        timeStart = millis();
        pressed = i;
        holding = false;
        break;
      }
    }
  }
  else {
    // If holding
    if (digitalRead(buttonPin[pressed]) == LOW && millis() - timeStart > holdDelay) {
      holding = true;
      Serial.print("Holding button ");
      Serial.println(pressed);
    }
    // If released
    else if (digitalRead(buttonPin[pressed]) == HIGH) {
      if (!holding) {
        Serial.print("Released button ");
        Serial.println(pressed);
      }
      pressed = -1;
    }
  }
}