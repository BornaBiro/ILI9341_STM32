#include "SPI.h"
#include "Adafruit_GFX.h"
#include "myILI9341.h"

// For the Adafruit shield, these are the default.
#define TFT_DC 9
#define TFT_CS 10

// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
Adafruit_ILI9341 display = Adafruit_ILI9341(TFT_CS, TFT_DC);
unsigned long t1, t2;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  display.begin();
  display.fillRect(0, 0, 320, 240, ILI9341_WHITE);
  display.display();
  display.fillRect(0, 0, 320, 240, ILI9341_BLACK);
  display.fillRect(100, 100, 100, 100, ILI9341_PINK);
  t1 = micros();
  display.display();
  t2 = micros();
  Serial.println(t2 - t1, DEC);
}
int k = 0;
void loop() {
  // put your main code here, to run repeatedly:
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(4);
  display.setTextColor(ILI9341_BLUE);
  display.print("Broj:");
  display.print(k, DEC);
  display.display();
  k++;
  delay(1000);
}
