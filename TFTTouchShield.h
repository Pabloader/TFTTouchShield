#ifndef _ADAFRUIT_TFTLCD_H_
#define _ADAFRUIT_TFTLCD_H_

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Adafruit_GFX.h>

#define __PRESSURE 10

#define	TFT_BLACK   0x0000
#define	TFT_BLUE    0x001F
#define	TFT_RED     0xF800
#define	TFT_GREEN   0x07E0
#define TFT_CYAN    0x07FF
#define TFT_MAGENTA 0xF81F
#define TFT_YELLOW  0xFFE0
#define TFT_WHITE   0xFFFF

class TouchPoint {
    public:
    int x, y, z;

    public:
    TouchPoint(void);
    TouchPoint(int x, int y, int z);
    bool isTouching();
    bool operator==(TouchPoint);
    bool operator!=(TouchPoint);
};

class TFTTouchShield : public Adafruit_GFX {
 public:

  TFTTouchShield(uint8_t cs = A3, uint8_t cd = A2, uint8_t wr = A1, uint8_t rd = A0, uint8_t rst = A4);

  void     begin();
  void     setupTouchScreen(
      unsigned char xp = 7, 
      unsigned char yp = A2, 
      unsigned char xm = A1, 
      unsigned char ym = 6);
  void     drawPixel(int16_t x, int16_t y, uint16_t color);
  void     drawFastHLine(int16_t x0, int16_t y0, int16_t w, uint16_t color);
  void     drawFastVLine(int16_t x0, int16_t y0, int16_t h, uint16_t color);
  void     fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t c);
  void     fillScreen(uint16_t color);
  void     reset(void);
  void     setRegisters8(uint8_t *ptr, uint8_t n);
  void     setRegisters16(uint16_t *ptr, uint8_t n);
  void     setRotation(uint8_t x);
       // These methods are public in order for BMP examples to work:
  void     setAddrWindow(int x1, int y1, int x2, int y2);
  void     pushColors(uint16_t *data, uint8_t len, boolean first);

  inline uint16_t color565(uint8_t r, uint8_t g, uint8_t b);
  uint16_t readPixel(int16_t x, int16_t y);
  uint16_t readID(void);
  uint32_t readReg(uint8_t r);
  bool isTouching(void);
  TouchPoint getPoint();

 private:
  unsigned char _yp, _ym, _xm, _xp;
  void     init(),
           // These items may have previously been defined as macros
           // in pin_magic.h.  If not, function versions are declared:
#ifndef write8
           write8(uint8_t value),
#endif
#ifndef setWriteDir
           setWriteDir(void),
#endif
#ifndef setReadDir
           setReadDir(void),
#endif
#ifndef writeRegister8
           writeRegister8(uint8_t a, uint8_t d),
#endif
#ifndef writeRegister16
           writeRegister16(uint16_t a, uint16_t d),
#endif
    writeRegister24(uint8_t a, uint32_t d),
    writeRegister32(uint8_t a, uint32_t d),
#ifndef writeRegisterPair
           writeRegisterPair(uint8_t aH, uint8_t aL, uint16_t d),
#endif
           setLR(void),
           flood(uint16_t color, uint32_t len);
  uint8_t  driver;

#ifndef read8
  uint8_t  read8fn(void);
  #define  read8isFunctionalized
#endif

#ifndef USE_ADAFRUIT_SHIELD_PINOUT

  #ifdef __AVR__
    volatile uint8_t *csPort    , *cdPort    , *wrPort    , *rdPort;
	uint8_t           csPinSet  ,  cdPinSet  ,  wrPinSet  ,  rdPinSet  ,
					  csPinUnset,  cdPinUnset,  wrPinUnset,  rdPinUnset,
					  _reset;
  #endif
  #if defined(__SAM3X8E__)
    Pio *csPort    , *cdPort    , *wrPort    , *rdPort;
	uint32_t          csPinSet  ,  cdPinSet  ,  wrPinSet  ,  rdPinSet  ,
					  csPinUnset,  cdPinUnset,  wrPinUnset,  rdPinUnset,
					  _reset;
  #endif
  
#endif
};

// For compatibility with sketches written for older versions of library.
// Color function name was changed to 'color565' for parity with 2.2" LCD
// library.
#define Color565 color565

#endif
