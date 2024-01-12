/*
  BlinkRGB

  Demonstrates usage of onboard RGB LED on some ESP dev boards.

  Calling digitalWrite(RGB_BUILTIN, HIGH) will use hidden RGB driver.
    
  RGBLedWrite demonstrates controll of each channel:
  void neopixelWrite(uint8_t pin, uint8_t red_val, uint8_t green_val, uint8_t blue_val)

  WARNING: After using digitalWrite to drive RGB LED it will be impossible to drive the same pin
    with normal HIGH/LOW level
*/
//#define RGB_BRIGHTNESS 64 // Change white brightness (max 255)

// the setup function runs once when you press reset or power the board

void setup() {
  // No need to initialize the RGB LED
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(14, LOW);   // Turn the RGB LED white
  digitalWrite(15, LOW);
  digitalWrite(16, LOW);
  delay(1000);
  digitalWrite(14, HIGH);   // Turn the RGB LED white
  digitalWrite(15, HIGH);
  digitalWrite(16, HIGH);
  delay(1000);

  digitalWrite(14, LOW);   // Turn the RGB LED white
  digitalWrite(15, HIGH);
  digitalWrite(16, HIGH);
  delay(1000);
  digitalWrite(14, HIGH);   // Turn the RGB LED white
  digitalWrite(15, LOW);
  digitalWrite(16, HIGH);
  delay(1000);
  digitalWrite(14, HIGH);   // Turn the RGB LED white
  digitalWrite(15, HIGH);
  digitalWrite(16, LOW);
  delay(1000);
  digitalWrite(14, HIGH);   // Turn the RGB LED white
  digitalWrite(15, HIGH);
  digitalWrite(16, HIGH);
  delay(1000);
}
