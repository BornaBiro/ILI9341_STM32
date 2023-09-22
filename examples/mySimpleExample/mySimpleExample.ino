#include "Adafruit_GFX.h"
#include "ILI9341_STM32.h"

#define TFT_DC 9
#define TFT_CS 10

// Use hardware SPI (D13 - SCK, D12 - MISO, D11 - MOSI)
Adafruit_ILI9341 display = Adafruit_ILI9341(TFT_CS, TFT_DC);
unsigned long t1, t2;
void setup()
{
  Serial.begin(115200);
  display.begin();
  display.fillRect(0, 0, 320, 240, ILI9341_WHITE);
  display.display();
  display.fillRect(0, 0, 320, 240, ILI9341_BLACK);
  display.fillRect(100, 100, 100, 100, ILI9341_PINK);
  t1 = micros();
  display.display();
  t2 = micros();
  
  // Print out one frame time in mmicroseconds
  Serial.println(t2 - t1, DEC);
}
int k = 0;
void loop() 
{
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
