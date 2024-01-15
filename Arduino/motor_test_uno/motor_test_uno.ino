// pin numbers
int pot1 = A0;
int pot2 = A1;
int mot1 = 9;
int mot2 = 10;

int fg1 = 2;
int fg2 = 3;

unsigned long time1 = 0;
int freq1 = 0;
volatile int counter1 = 0;
unsigned long time2 = 0;
int freq2 = 0;
volatile int counter2 = 0;

#include <TM1637Display.h>
#define CLK1 4
#define DIO1 5
TM1637Display display1(CLK1, DIO1);
#define CLK2 6
#define DIO2 7
TM1637Display display2(CLK2, DIO2);

void setup() {
  Serial.begin(9600);
  pinMode(pot1,INPUT_PULLUP);
  pinMode(pot2,INPUT_PULLUP);
  pinMode(mot1,OUTPUT);
  pinMode(mot2,OUTPUT);
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM10);
  TCCR1B = _BV(CS10);

  pinMode(fg1,INPUT_PULLUP);
  pinMode(fg2,INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(fg1), count1, RISING);
  attachInterrupt(digitalPinToInterrupt(fg2), count2, RISING);

  display1.setBrightness(7, true);
  display2.setBrightness(7, true);
}

void loop() {

  float in1 = 255.0 - analogRead(pot1) * 255.0 / 1023.0;
  in1 = 60.0 + in1 / 255.0 * 95.0;
  if (in1 <= 64.0) {
    in1 = 0.0;
  }
  else if (in1 >= 150.0) {
    in1 = 255.0;
  }

  float in2 = 255.0 - analogRead(pot2) * 255.0 / 1023.0;
  in2 = 60.0 + in2 / 255.0 * 95.0;
  if (in2 <= 64.0) {
    in2 = 0.0;
  }
  else if (in2 >= 150.0) {
    in2 = 255.0;
  }

  //float in1_max = 60.0 + (freq1 + 1000.0) / 8400.0 * 95.0;
  //in1 = min(in1, in1_max);
  //float in2_max = 60.0 + (freq2 + 1000.0) / 8400.0 * 95.0;
  //in2 = min(in2, in2_max);

  OCR1A = in1;
  OCR1B = in2;
  
  //analogWrite(mot1, in1);
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

  //freq1_filt = 0.8*freq1_filt + 0.2*freq1;
  display1.showNumberDec(freq1, false);

  //freq2_filt = 0.8*freq2_filt + 0.2*freq2;
  display2.showNumberDec(freq2, false);

  if (micros() - time1 >= 1000000) {
    time1 = micros();
    freq1 = 0;
    counter1 = 0;
  }
  if (micros() - time2 >= 1000000) {
    time2 = micros();
    freq2 = 0;
    counter2 = 0;
  }

  Serial.print(in1);
  Serial.print(", ");
  Serial.print(in2);
  Serial.print(", ");
//  Serial.print(in1_max);
//  Serial.print(", ");
//  Serial.print(in2_max);
//  Serial.print(", ");
  Serial.print(freq1);
  Serial.print(", ");
  Serial.print(freq2);
  Serial.print(", ");
  Serial.println(" ");
  delay(10);
}

void count1() {
  counter1 = ++counter1;
  if (counter1 >= 12) {
    unsigned long delta1 = micros() - time1;
    time1 = micros();
    freq1 = counter1 * 1000000.0 / delta1 / 12.0 * 60;
    counter1 = 0;
  }
}

void count2() {
  counter2 = ++counter2;
  if (counter2 >= 12) {
    unsigned long delta2 = micros() - time2;
    time2 = micros();
    freq2 = counter2 * 1000000.0 / delta2 / 12.0 * 60;
    counter2 = 0;
  }
}