// demoOled.h
// to be included by .ino and .cpp files

#include <Adafruit_SSD1306.h>
#include <npsoe_iot_kit.h>
#define SCR_WIDTH      128    // OLED display width, in pixels
#define SCR_HEIGHT     64     // OLED display height, in pixels
#define TXT_WIDTH      6
#define TXT_HEIGHT     8
#define TXT_SCR_WIDTH  (SCR_WIDTH/TXT_WIDTH)
#define TXT_SCR_HEIGHT (SCR_HEIGHT/TXT_HEIGHT)
// -----------------------------------
// These are found in screen.cpp shared with other files
extern Adafruit_SSD1306 oled;
void printTemplate(bool printDummyToo = false); // prototype with default
void oledInit();
void hang();     // hang system with red LED blinking
void oledSetTextCursor(uint8_t x, uint8_t y);
