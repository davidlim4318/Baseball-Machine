// Complete Instructions to Get and Change ESP MAC Address: https://RandomNerdTutorials.com/get-change-esp32-esp8266-mac-address-arduino/

#include "WiFi.h"

void setup(){
  Serial.begin(9600);
  WiFi.mode(WIFI_MODE_STA);
}
 
void loop(){
  Serial.println(WiFi.macAddress());
  delay(1000);
}