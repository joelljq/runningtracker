// screen.cpp

#include "demoOled.h"

#define SCR_ADDRESS 0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 oled(SCR_WIDTH, SCR_HEIGHT);
// -----------------------------------
void oledInit() {
  if (oled.begin(SSD1306_SWITCHCAPVCC, SCR_ADDRESS)) {
    oled.clearDisplay();
    oled.setTextSize(1);              // Normal 1:1 pixel scale
    oled.setTextColor(WHITE, BLACK);  // Draw white text with black background
  }
  else { // problem, hang!
    hang();
  }
}
// -----------------------------------
// set cursor for default text font each character takes up 8x6 pixels
// x:0 to 20, y:0 to 7
void oledSetTextCursor(uint8_t x, uint8_t y) {
  if (x > (TXT_SCR_WIDTH - 1) || y > (TXT_SCR_HEIGHT - 1)) return; // out of bound
  oled.setCursor(x * TXT_WIDTH, y * TXT_HEIGHT);
}
// -----------------------------------
//  012345678901234567890
//0 My Application
//1 Sensors readings
//2 Rotary:   4095 = 3.3V
//3 Accel   X    Y    Z
//4       1.0  0.0  0.0 g
//5 Temperature:  25.6  C
//6 Humidity:      100  %
//7 Ultrasonic:    100 cm

void printTemplate(bool printDummyToo) {
  oledSetTextCursor(0, 0);
  oled.print("My Application");
  oledSetTextCursor(0, 1);
  oled.print("Sensors Readings");
  oledSetTextCursor(0, 2);
  oled.print("Rotary:");
  oledSetTextCursor(15, 2);
  oled.print('=');
  oledSetTextCursor(20, 2);
  oled.print('V');
  oledSetTextCursor(0, 3);
  oled.print("Accel   X    Y    Z");
  oledSetTextCursor(20, 4);
  oled.print('g');
  oledSetTextCursor(0, 5);
  oled.print("Temperature:        C");
  oledSetTextCursor(0, 6);
  oled.print("Humidity:           %");
  oledSetTextCursor(0, 7);
  oled.print("Ultrasonic:        cm");
  if (printDummyToo) {
    oledSetTextCursor(5, 4);      // print accelerometer
    oled.print(" ?.?  ?.?  ?.?"); // DUMMY - each 4 columns with 1 decimal place
    oledSetTextCursor(14, 5);     // print temperature
    oled.print("??.?");           // DUMMY - 4 columns with 1 decimal place
    oledSetTextCursor(15, 6);     // print humidity
    oled.print("???");            // DUMMY - 3 columns with no decimal place
    oledSetTextCursor(13, 7);     // print ultrasonic value
    oled.print("  ???");          // DUMMY - 5 columns integer
  }
  oled.display();    // after "printing", need .display() to push to screen
}
// -----------------------------------
void hang() {
  pinMode(LED_R, OUTPUT);
  Serial.print("\nSystem hang");
  for (;;) {
    digitalWrite(LED_R, HIGH);
    delay(250);
    digitalWrite(LED_R, LOW);
    delay(250);
  }
}
// -----------------------------------
