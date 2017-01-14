/*********************************************************************
This is a tiny graphics driver library for Adafruit SSD1306 
monochrome OLED displays (128x64) to be used over hardware SPI. 
It stores the screen buffer, sets a pixel, or clears the screen. 

This library only requires 1264 bytes of flash on top of the basic 
minimum!

Based off of Adafruit's provided libraries. Tidied up by Alice Wang.
*********************************************************************/

#ifndef _SSD1306_GFX_Lite_H_
#define _SSD1306_GFX_Lite_H_

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#if defined(__SAM3X8E__)
 typedef volatile RwReg PortReg;
 typedef uint32_t PortMask;
 #define HAVE_PORTREG
#elif defined(ARDUINO_ARCH_SAMD)
// not supported
#elif defined(ESP8266) || defined(ARDUINO_STM32_FEATHER)
  typedef volatile uint32_t PortReg;
  typedef uint32_t PortMask;
#else
  typedef volatile uint8_t PortReg;
  typedef uint8_t PortMask;
 #define HAVE_PORTREG
#endif

#include <SPI.h>

#define BLACK 0
#define WHITE 1

#define SSD1306_I2C_ADDRESS 0x3C  // 011110+SA0+RW - 0x3C or 0x3D

#define SSD1306_LCDWIDTH 128
#define SSD1306_LCDHEIGHT 64

#define SSD1306_SETCONTRAST 0x81
#define SSD1306_DISPLAYALLON_RESUME 0xA4
#define SSD1306_DISPLAYALLON 0xA5
#define SSD1306_NORMALDISPLAY 0xA6
#define SSD1306_INVERTDISPLAY 0xA7
#define SSD1306_DISPLAYOFF 0xAE
#define SSD1306_DISPLAYON 0xAF

#define SSD1306_SETDISPLAYOFFSET 0xD3
#define SSD1306_SETCOMPINS 0xDA

#define SSD1306_SETVCOMDETECT 0xDB

#define SSD1306_SETDISPLAYCLOCKDIV 0xD5
#define SSD1306_SETPRECHARGE 0xD9

#define SSD1306_SETMULTIPLEX 0xA8

#define SSD1306_SETLOWCOLUMN 0x00
#define SSD1306_SETHIGHCOLUMN 0x10

#define SSD1306_SETSTARTLINE 0x40

#define SSD1306_MEMORYMODE 0x20
#define SSD1306_COLUMNADDR 0x21
#define SSD1306_PAGEADDR   0x22

#define SSD1306_COMSCANINC 0xC0
#define SSD1306_COMSCANDEC 0xC8

#define SSD1306_SEGREMAP 0xA0

#define SSD1306_CHARGEPUMP 0x8D

#define SSD1306_EXTERNALVCC 0x1
#define SSD1306_SWITCHCAPVCC 0x2

#define SSD1306_DEACTIVATE_SCROLL 0x2E

class SSD1306_GFX_Lite {
 public:
  SSD1306_GFX_Lite(int8_t DC, int8_t RST, int8_t CS);
  void begin(uint8_t switchvcc = SSD1306_SWITCHCAPVCC, uint8_t i2caddr = SSD1306_I2C_ADDRESS, bool reset=true);
  void ssd1306_command(uint8_t c);

  void clearDisplay(void);
  void display();
  void drawPixel(int16_t x, int16_t y, uint16_t color);

 private:
  int8_t _i2caddr, _vccstate, sid, sclk, dc, rst, cs;

#ifdef HAVE_PORTREG
  PortReg *mosiport, *clkport, *csport, *dcport;
  PortMask mosipinmask, clkpinmask, cspinmask, dcpinmask;
#endif

};

#endif /* _SSD1306_GFX_Lite_H_ */
