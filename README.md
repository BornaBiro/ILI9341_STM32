# ILI9341_STM32
Library is designed to work with STM32H743 MCU (Nucleo-H743ZI2 board) and ILI9341 LCD TFT with SPI communication with Arduino IDE.
In order to use this libarary, you have to install STM32Duino Arduino Core and Adafruit GFX library.
This is heavily modified Adafruit_ILI9341 and Adafruit_SPITFT library.

THIS LIBRARY IS STILL IN PROGRESS so not all functons work properly!

Also, add 330 Ohm resistor in series with SPI SCK (D13) and MOSI (D11) and use as short wires as possible!!!

TFT DC is connected by default on D9 and TFT CS is connected on D10. MISO is connected on D12 but it's not used here.

This library uses frame buffer to store frame in RAM so it uses a lot of RAM (~150k) and uses global refresh.
Refresh rate is ~ 27FPS.

TODO:
- Fix rotation bug
- Use DMA instead Interrupts
