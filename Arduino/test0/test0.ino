int IN1 = D2;
int IN2 = D3;
int LED = D4;
int value = 0;

void setup() {
  Serial.begin(19200);
  pinMode(IN1,INPUT_PULLUP);
  pinMode(IN2,INPUT_PULLUP);
  pinMode(LED,OUTPUT);
  analogWriteResolution(10);
  analogWriteFrequency(25000);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (digitalRead(IN1) == 0) {
    value = --value;
  }
  if (digitalRead(IN2) == 0) {
    value = ++value;
  }
  value = constrain(value, 0, 1023);
  Serial.println(value);
  analogWrite(LED, value);
  delay(1);
}