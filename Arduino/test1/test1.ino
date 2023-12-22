// pin numbers
int pot1 = A0;
int pot2 = A1;
int mot1 = 9;
int mot2 = 10;

int fg1 = 2;
int fg2 = 3;

unsigned long time = 0;
float freq = 0;
volatile int counter = 0;

#include <TM1637Display.h>
#define CLK 12
#define DIO 13
TM1637Display display(CLK, DIO);

void setup() {
  Serial.begin(9600);
  pinMode(mot1,OUTPUT);
  pinMode(mot2,OUTPUT);
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM10);
  TCCR1B = _BV(CS10);

  pinMode(fg1,INPUT);
  pinMode(fg2,INPUT);

  attachInterrupt(digitalPinToInterrupt(fg1), count, RISING);

  display.setBrightness(7, true);
}

void loop() {
  float in1 = analogRead(pot1) * 255.0 / 1023.0;
  in1 = 60.0 + in1 / 255.0 * 95.0;
  if (in1 <= 62.0) {
    in1 = 0.0;
  }
  else if (in1 >= 152.0) {
    in1 = 255.0;
  }
  OCR1A = in1;
  //analogWrite(mot1, in1);

  float in2 = analogRead(pot2) * 255.0 / 1023.0;
  OCR1B = in2;
  //analogWrite(mos2, in2);

  //float ontime1 = pulseIn(fg1,HIGH,10000);
  //float offtime1 = pulseIn(fg1,LOW,10000);
  //float freq = 1000000.0 / (2 * offtime1);

  //unsigned long delta = micros() - time;
  //if (counter >= 10) {
  //  time = micros();
  //  freq = counter * 1000000.0 / delta;
  //  counter = 0;
  //}
  //if (delta > 1000000.0) {
  //  time = micros();
  //  freq = 0;
  //  counter = 0;
  //}
  //Serial.println(freq);
  
  unsigned long delta = micros() - time;
  time = micros();
  int freq = counter * 1000000.0 / delta / 12.0 * 60;
  counter = 0;
  Serial.println(freq);
  display.showNumberDec(freq, false);
  delay(100);
}

void count() {
  counter = ++counter;
}