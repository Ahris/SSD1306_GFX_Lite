/*********************************************************************
This is a tiny graphics driver library for Adafruit SSD1306 
monochrome OLED displays (128x64) to be used over hardware SPI. 
It stores the screen buffer, sets a pixel, or clears the screen. 

This library only requires 1264 bytes of flash on top of the basic 
minimum!

Based off of Adafruit's provided libraries. Tidied up by Alice Wang.
*********************************************************************/

#ifdef __AVR__
  #include <avr/pgmspace.h>
#elif defined(ESP8266)
 #include <pgmspace.h>
#else
 #define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#endif

#if !defined(__ARM_ARCH) && !defined(ENERGIA) && !defined(ESP8266)
 #include <util/delay.h>
#endif

#include <SPI.h>
#include "SSD1306_GFX_Lite.h"

// the memory buffer for the LCD
static uint8_t buffer[SSD1306_LCDHEIGHT * SSD1306_LCDWIDTH / 8] = {};

// constructor for hardware SPI - we indicate DataCommand, ChipSelect, Reset
SSD1306_GFX_Lite::SSD1306_GFX_Lite(int8_t DC, int8_t RST, int8_t CS) {
  dc = DC;
  rst = RST;
  cs = CS;
}

void SSD1306_GFX_Lite::begin(uint8_t vccstate, uint8_t i2caddr, bool reset) {
  _vccstate = vccstate;
  _i2caddr = i2caddr;

  // set pin directions
  if (sid != -1){
    pinMode(dc, OUTPUT);
    pinMode(cs, OUTPUT);
#ifdef HAVE_PORTREG
    csport      = portOutputRegister(digitalPinToPort(cs));
    cspinmask   = digitalPinToBitMask(cs);
    dcport      = portOutputRegister(digitalPinToPort(dc));
    dcpinmask   = digitalPinToBitMask(dc);
#endif

    SPI.begin();
#ifdef SPI_HAS_TRANSACTION
    SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
#else
    SPI.setClockDivider (4);
#endif
  }
  if ((reset) && (rst >= 0)) {
    // Setup reset pin direction (used by both SPI and I2C)
    pinMode(rst, OUTPUT);
    digitalWrite(rst, HIGH);
    // VDD (3.3V) goes high at start, lets just chill for a ms
    delay(1);
    // bring reset low
    digitalWrite(rst, LOW);
    // wait 10ms
    delay(10);
    // bring out of reset
    digitalWrite(rst, HIGH);
    // turn on VCC (9V?)
  }

  // Init sequence
  ssd1306_command(SSD1306_DISPLAYOFF);                    // 0xAE
  ssd1306_command(SSD1306_SETDISPLAYCLOCKDIV);            // 0xD5
  ssd1306_command(0x80);                                  // the suggested ratio 0x80

  ssd1306_command(SSD1306_SETMULTIPLEX);                  // 0xA8
  ssd1306_command(SSD1306_LCDHEIGHT - 1);

  ssd1306_command(SSD1306_SETDISPLAYOFFSET);              // 0xD3
  ssd1306_command(0x0);                                   // no offset
  ssd1306_command(SSD1306_SETSTARTLINE | 0x0);            // line #0
  ssd1306_command(SSD1306_CHARGEPUMP);                    // 0x8D
  if (vccstate == SSD1306_EXTERNALVCC) { 
    ssd1306_command(0x10); 
  } else { 
    ssd1306_command(0x14); 
  }
  ssd1306_command(SSD1306_MEMORYMODE);                    // 0x20
  ssd1306_command(0x00);                                  // 0x0 act like ks0108
  ssd1306_command(SSD1306_SEGREMAP | 0x1);
  ssd1306_command(SSD1306_COMSCANDEC);

  ssd1306_command(SSD1306_SETCOMPINS);                    // 0xDA
  ssd1306_command(0x12);
  ssd1306_command(SSD1306_SETCONTRAST);                   // 0x81
  if (vccstate == SSD1306_EXTERNALVCC) { 
    ssd1306_command(0x9F); 
  } else { 
    ssd1306_command(0xCF); 
  }

  ssd1306_command(SSD1306_SETPRECHARGE);                  // 0xd9
  if (vccstate == SSD1306_EXTERNALVCC) { 
    ssd1306_command(0x22); 
  } else { 
    ssd1306_command(0xF1); 
  }
  ssd1306_command(SSD1306_SETVCOMDETECT);                 // 0xDB
  ssd1306_command(0x40);
  ssd1306_command(SSD1306_DISPLAYALLON_RESUME);           // 0xA4
  ssd1306_command(SSD1306_NORMALDISPLAY);                 // 0xA6

  ssd1306_command(SSD1306_DEACTIVATE_SCROLL);

  //--turn on oled panel
  ssd1306_command(SSD1306_DISPLAYON); 
}

void SSD1306_GFX_Lite::ssd1306_command(uint8_t c) {
  if (sid != -1) {
    // SPI
#ifdef HAVE_PORTREG
    *csport |= cspinmask;
    *dcport &= ~dcpinmask;
    *csport &= ~cspinmask;
#else
    digitalWrite(cs, HIGH);
    digitalWrite(dc, LOW);
    digitalWrite(cs, LOW);
#endif
    SPI.transfer(c);
#ifdef HAVE_PORTREG
    *csport |= cspinmask;
#else
    digitalWrite(cs, HIGH);
#endif
  }
}

void SSD1306_GFX_Lite::display(void) {
  ssd1306_command(SSD1306_COLUMNADDR);
  ssd1306_command(0);   // Column start address (0 = reset)
  ssd1306_command(SSD1306_LCDWIDTH-1); // Column end address (127 = reset)
  ssd1306_command(SSD1306_PAGEADDR);
  ssd1306_command(0); // Page start address (0 = reset)
  ssd1306_command(7); // Page end address

  if (sid != -1) {
    // SPI
#ifdef HAVE_PORTREG
    *csport |= cspinmask;
    *dcport |= dcpinmask;
    *csport &= ~cspinmask;
#else
    digitalWrite(cs, HIGH);
    digitalWrite(dc, HIGH);
    digitalWrite(cs, LOW);
#endif

    for (uint16_t i=0; i<(SSD1306_LCDWIDTH*SSD1306_LCDHEIGHT/8); ++i) {
      SPI.transfer(buffer[i]);
    }
#ifdef HAVE_PORTREG
    *csport |= cspinmask;
#else
    digitalWrite(cs, HIGH);
#endif
  }
}

// x == col, y == row
void SSD1306_GFX_Lite::drawPixel(int16_t x, int16_t y, uint16_t color) {
  if ((x < 0) || (x >= SSD1306_LCDWIDTH) || (y < 0) || (y >= SSD1306_LCDHEIGHT)) {
    return;
  }

  switch (color)
  {
    case WHITE:   buffer[x+ (y/8)*SSD1306_LCDWIDTH] |=  (1 << (y&7)); break;
    case BLACK:   buffer[x+ (y/8)*SSD1306_LCDWIDTH] &= ~(1 << (y&7)); break;
  }
}

void SSD1306_GFX_Lite::clearDisplay(void) {
  memset(buffer, 0, (SSD1306_LCDWIDTH*SSD1306_LCDHEIGHT/8));
}
