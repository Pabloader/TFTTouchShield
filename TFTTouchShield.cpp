// Graphics library by ladyada/adafruit with init code from Rossum
// MIT license

#if defined(__SAM3X8E__)
	#include <include/pio.h>
    #define PROGMEM
    #define pgm_read_byte(addr) (*(const unsigned char *)(addr))
    #define pgm_read_word(addr) (*(const unsigned short *)(addr))
#endif
#ifdef __AVR__
	#include <avr/pgmspace.h>
#endif
#include "pins_arduino.h"
#include "wiring_private.h"
#include "TFTTouchShield.h"
#include "pin_magic.h"

#define TFTWIDTH   240
#define TFTHEIGHT  320

#define NUMSAMPLES 2		// sample number
#define COMP       2
#define AVERAGE    1
#define RXPLATE    300

#include "registers.h"

// Constructor for breakout board (configurable LCD control lines).
// Can still use this w/shield, but parameters are ignored.
TFTTouchShield::TFTTouchShield(
  uint8_t cs, uint8_t cd, uint8_t wr, uint8_t rd, uint8_t reset) :
  Adafruit_GFX(TFTWIDTH, TFTHEIGHT) {

  _reset     = reset;
  #ifdef __AVR__
    csPort     = portOutputRegister(digitalPinToPort(cs));
    cdPort     = portOutputRegister(digitalPinToPort(cd));
    wrPort     = portOutputRegister(digitalPinToPort(wr));
    rdPort     = portOutputRegister(digitalPinToPort(rd));
  #endif
  #if defined(__SAM3X8E__)
    csPort     = digitalPinToPort(cs);
    cdPort     = digitalPinToPort(cd);
    wrPort     = digitalPinToPort(wr);
    rdPort     = digitalPinToPort(rd);
  #endif
  csPinSet   = digitalPinToBitMask(cs);
  cdPinSet   = digitalPinToBitMask(cd);
  wrPinSet   = digitalPinToBitMask(wr);
  rdPinSet   = digitalPinToBitMask(rd);
  csPinUnset = ~csPinSet;
  cdPinUnset = ~cdPinSet;
  wrPinUnset = ~wrPinSet;
  rdPinUnset = ~rdPinSet;
  #ifdef __AVR__
    *csPort   |=  csPinSet; // Set all control bits to HIGH (idle)
    *cdPort   |=  cdPinSet; // Signals are ACTIVE LOW
    *wrPort   |=  wrPinSet;
    *rdPort   |=  rdPinSet;
  #endif
  #if defined(__SAM3X8E__)
    csPort->PIO_SODR  |=  csPinSet; // Set all control bits to HIGH (idle)
    cdPort->PIO_SODR  |=  cdPinSet; // Signals are ACTIVE LOW
    wrPort->PIO_SODR  |=  wrPinSet;
    rdPort->PIO_SODR  |=  rdPinSet;
  #endif
  pinMode(cs, OUTPUT);    // Enable outputs
  pinMode(cd, OUTPUT);
  pinMode(wr, OUTPUT);
  pinMode(rd, OUTPUT);
  if (reset) {
    digitalWrite(reset, HIGH);
    pinMode(reset, OUTPUT);
  }

  init();
}


// Initialization common to both shield & breakout configs
void TFTTouchShield::init(void) {

  setWriteDir(); // Set up LCD data port(s) for WRITE operations

  rotation  = 0;
  cursor_y  = cursor_x = 0;
  textsize  = 1;
  textcolor = 0xFFFF;
  _width    = TFTWIDTH;
  _height   = TFTHEIGHT;
}

// Initialization command tables for different LCD controllers
#define TFTLCD_DELAY 0xFF

static const uint16_t ILI932x_regValues[] PROGMEM = {
  ILI932X_START_OSC        , 0x0001, // Start oscillator
  TFTLCD_DELAY             , 50,     // 50 millisecond delay
  ILI932X_DRIV_OUT_CTRL    , 0x0100,
  ILI932X_DRIV_WAV_CTRL    , 0x0700,
  ILI932X_ENTRY_MOD        , 0x1030,
  ILI932X_RESIZE_CTRL      , 0x0000,
  ILI932X_DISP_CTRL2       , 0x0202,
  ILI932X_DISP_CTRL3       , 0x0000,
  ILI932X_DISP_CTRL4       , 0x0000,
  ILI932X_RGB_DISP_IF_CTRL1, 0x0,
  ILI932X_FRM_MARKER_POS   , 0x0,
  ILI932X_RGB_DISP_IF_CTRL2, 0x0,
  ILI932X_POW_CTRL1        , 0x0000,
  ILI932X_POW_CTRL2        , 0x0007,
  ILI932X_POW_CTRL3        , 0x0000,
  ILI932X_POW_CTRL4        , 0x0000,
  TFTLCD_DELAY             , 200,
  ILI932X_POW_CTRL1        , 0x1690,
  ILI932X_POW_CTRL2        , 0x0227,
  TFTLCD_DELAY             , 50,
  ILI932X_POW_CTRL3        , 0x001A,
  TFTLCD_DELAY             , 50,
  ILI932X_POW_CTRL4        , 0x1800,
  ILI932X_POW_CTRL7        , 0x002A,
  TFTLCD_DELAY             , 50,
  ILI932X_GAMMA_CTRL1      , 0x0000,
  ILI932X_GAMMA_CTRL2      , 0x0000,
  ILI932X_GAMMA_CTRL3      , 0x0000,
  ILI932X_GAMMA_CTRL4      , 0x0206,
  ILI932X_GAMMA_CTRL5      , 0x0808,
  ILI932X_GAMMA_CTRL6      , 0x0007,
  ILI932X_GAMMA_CTRL7      , 0x0201,
  ILI932X_GAMMA_CTRL8      , 0x0000,
  ILI932X_GAMMA_CTRL9      , 0x0000,
  ILI932X_GAMMA_CTRL10     , 0x0000,
  ILI932X_GRAM_HOR_AD      , 0x0000,
  ILI932X_GRAM_VER_AD      , 0x0000,
  ILI932X_HOR_START_AD     , 0x0000,
  ILI932X_HOR_END_AD       , 0x00EF,
  ILI932X_VER_START_AD     , 0X0000,
  ILI932X_VER_END_AD       , 0x013F,
  ILI932X_GATE_SCAN_CTRL1  , 0xA700, // Driver Output Control (R60h)
  ILI932X_GATE_SCAN_CTRL2  , 0x0003, // Driver Output Control (R61h)
  ILI932X_GATE_SCAN_CTRL3  , 0x0000, // Driver Output Control (R62h)
  ILI932X_PANEL_IF_CTRL1   , 0X0010, // Panel Interface Control 1 (R90h)
  ILI932X_PANEL_IF_CTRL2   , 0X0000,
  ILI932X_PANEL_IF_CTRL3   , 0X0003,
  ILI932X_PANEL_IF_CTRL4   , 0X1100,
  ILI932X_PANEL_IF_CTRL5   , 0X0000,
  ILI932X_PANEL_IF_CTRL6   , 0X0000,
  ILI932X_DISP_CTRL1       , 0x0133, // Main screen turn on
};

void TFTTouchShield::begin() {
  uint8_t i = 0;

  reset();
  delay(200);

  uint16_t a, d;
  CS_ACTIVE;
  writeRegister8(ILI9341_SOFTRESET, 0);
  delay(50);
  writeRegister8(ILI9341_DISPLAYOFF, 0);

  writeRegister8(ILI9341_POWERCONTROL1, 0x23);
  writeRegister8(ILI9341_POWERCONTROL2, 0x10);
  writeRegister16(ILI9341_VCOMCONTROL1, 0x2B2B);
  writeRegister8(ILI9341_VCOMCONTROL2, 0xC0);
  writeRegister8(ILI9341_MEMCONTROL, ILI9341_MADCTL_MY | ILI9341_MADCTL_BGR);
  writeRegister8(ILI9341_PIXELFORMAT, 0x55);
  writeRegister16(ILI9341_FRAMECONTROL, 0x001B);
  
  writeRegister8(ILI9341_ENTRYMODE, 0x07);
  /* writeRegister32(ILI9341_DISPLAYFUNC, 0x0A822700);*/

  writeRegister8(ILI9341_SLEEPOUT, 0);
  delay(150);
  writeRegister8(ILI9341_DISPLAYON, 0);
  delay(500);
  setAddrWindow(0, 0, TFTWIDTH-1, TFTHEIGHT-1);
}

void TFTTouchShield::reset(void) {

  CS_IDLE;
//  CD_DATA;
  WR_IDLE;
  RD_IDLE;

  if (_reset) {
    digitalWrite(_reset, LOW);
    delay(2);
    digitalWrite(_reset, HIGH);
  }

  // Data transfer sync
  CS_ACTIVE;
  CD_COMMAND;
  write8(0x00);
  for(uint8_t i=0; i<3; i++) WR_STROBE; // Three extra 0x00s
  CS_IDLE;
}

// Sets the LCD address window (and address counter, on 932X).
// Relevant to rect/screen fills and H/V lines.  Input coordinates are
// assumed pre-sorted (e.g. x2 >= x1).
void TFTTouchShield::setAddrWindow(int x1, int y1, int x2, int y2) {
  CS_ACTIVE;
  uint32_t t;

  t = x1;
  t <<= 16;
  t |= x2;
  writeRegister32(ILI9341_COLADDRSET, t);  
  t = y1;
  t <<= 16;
  t |= y2;
  writeRegister32(ILI9341_PAGEADDRSET, t);

  CS_IDLE;
}

// Unlike the 932X drivers that set the address window to the full screen
// by default (using the address counter for drawPixel operations), the
// 7575 needs the address window set on all graphics operations.  In order
// to save a few register writes on each pixel drawn, the lower-right
// corner of the address window is reset after most fill operations, so
// that drawPixel only needs to change the upper left each time.
void TFTTouchShield::setLR(void) {
  CS_ACTIVE;
  writeRegisterPair(HX8347G_COLADDREND_HI, HX8347G_COLADDREND_LO, _width  - 1);
  writeRegisterPair(HX8347G_ROWADDREND_HI, HX8347G_ROWADDREND_LO, _height - 1);
  CS_IDLE;
}

// Fast block fill operation for fillScreen, fillRect, H/V line, etc.
// Requires setAddrWindow() has previously been called to set the fill
// bounds.  'len' is inclusive, MUST be >= 1.
void TFTTouchShield::flood(uint16_t color, uint32_t len) {
  uint16_t blocks;
  uint8_t  i, hi = ~color >> 8,
              lo = ~color;

  CS_ACTIVE;
  CD_COMMAND;
  write8(0x2C);

  // Write first pixel normally, decrement counter by 1
  CD_DATA;
  write8(hi);
  write8(lo);
  len--;

  blocks = (uint16_t)(len / 64); // 64 pixels/block
  if (hi == lo) {
    // High and low bytes are identical.  Leave prior data
    // on the port(s) and just toggle the write strobe.
    while (blocks--) {
      i = 16; // 64 pixels/block / 4 pixels/pass
      do {
        WR_STROBE; WR_STROBE; WR_STROBE; WR_STROBE; // 2 bytes/pixel
        WR_STROBE; WR_STROBE; WR_STROBE; WR_STROBE; // x 4 pixels
      } while(--i);
    }
    // Fill any remaining pixels (1 to 64)
    for (i = (uint8_t)len & 63; i--; ) {
      WR_STROBE;
      WR_STROBE;
    }
  } else {
    while (blocks--) {
      i = 16; // 64 pixels/block / 4 pixels/pass
      do {
        write8(hi); write8(lo); write8(hi); write8(lo);
        write8(hi); write8(lo); write8(hi); write8(lo);
      } while(--i);
    }
    for(i = (uint8_t)len & 63; i--; ) {
      write8(hi);
      write8(lo);
    }
  }
  CS_IDLE;
}

void TFTTouchShield::drawFastHLine(int16_t x, int16_t y, int16_t length,
  uint16_t color)
{
  int16_t x2;

  // Initial off-screen clipping
  if((length <= 0     ) ||
     (y      <  0     ) || ( y                  >= _height) ||
     (x      >= _width) || ((x2 = (x+length-1)) <  0      )) return;

  if(x < 0) {        // Clip left
    length += x;
    x       = 0;
  }
  if(x2 >= _width) { // Clip right
    x2      = _width - 1;
    length  = x2 - x + 1;
  }

  setAddrWindow(x, y, x2, y);
  flood(color, length);
  setLR();
}

void TFTTouchShield::drawFastVLine(int16_t x, int16_t y, int16_t length,
  uint16_t color)
{
  int16_t y2;

  // Initial off-screen clipping
  if((length <= 0      ) ||
     (x      <  0      ) || ( x                  >= _width) ||
     (y      >= _height) || ((y2 = (y+length-1)) <  0     )) return;
  if(y < 0) {         // Clip top
    length += y;
    y       = 0;
  }
  if(y2 >= _height) { // Clip bottom
    y2      = _height - 1;
    length  = y2 - y + 1;
  }

  setAddrWindow(x, y, x, y2);
  flood(color, length);
  setLR();
}

void TFTTouchShield::fillRect(int16_t x1, int16_t y1, int16_t w, int16_t h, 
  uint16_t fillcolor) {
  int16_t  x2, y2;

  // Initial off-screen clipping
  if( (w            <= 0     ) ||  (h             <= 0      ) ||
      (x1           >= _width) ||  (y1            >= _height) ||
     ((x2 = x1+w-1) <  0     ) || ((y2  = y1+h-1) <  0      )) return;
  if(x1 < 0) { // Clip left
    w += x1;
    x1 = 0;
  }
  if(y1 < 0) { // Clip top
    h += y1;
    y1 = 0;
  }
  if(x2 >= _width) { // Clip right
    x2 = _width - 1;
    w  = x2 - x1 + 1;
  }
  if(y2 >= _height) { // Clip bottom
    y2 = _height - 1;
    h  = y2 - y1 + 1;
  }

  setAddrWindow(x1, y1, x2, y2);
  flood(fillcolor, (uint32_t)w * (uint32_t)h);
  setLR();
}

void TFTTouchShield::fillScreen(uint16_t color) {
  setAddrWindow(0, 0, _width - 1, _height - 1);
  flood(color, (long)TFTWIDTH * (long)TFTHEIGHT);
}

void TFTTouchShield::drawPixel(int16_t x, int16_t y, uint16_t color) {

  // Clip
  if((x < 0) || (y < 0) || (x >= _width) || (y >= _height)) return;

  CS_ACTIVE;
  
  setAddrWindow(x, y, _width-1, _height-1);
  CS_ACTIVE;
  CD_COMMAND; 
  write8(0x2C);
  CD_DATA; 
  write8(~color >> 8); write8(~color);

  CS_IDLE;
}

// Issues 'raw' an array of 16-bit color values to the LCD; used
// externally by BMP examples.  Assumes that setWindowAddr() has
// previously been set to define the bounds.  Max 255 pixels at
// a time (BMP examples read in small chunks due to limited RAM).
void TFTTouchShield::pushColors(uint16_t *data, uint8_t len, boolean first) {
  uint16_t color;
  uint8_t  hi, lo;
  CS_ACTIVE;
  if (first == true) { // Issue GRAM write command only on first call
    CD_COMMAND;
    write8(0x2C);
  }
  CD_DATA;
  while(len--) {
    color = *data++;
    hi    = color >> 8; // Don't simplify or merge these
    lo    = color;      // lines, there's macro shenanigans
    write8(hi);         // going on.
    write8(lo);
  }
  CS_IDLE;
}

void TFTTouchShield::setRotation(uint8_t x) {

  // Call parent rotation func first -- sets up rotation flags, etc.
  Adafruit_GFX::setRotation(x);
  // Then perform hardware-specific rotation operations...

  CS_ACTIVE;
   uint16_t t;

   switch (rotation) {
    case 2:
      t = ILI9341_MADCTL_MX | ILI9341_MADCTL_BGR;
      break;
    case 3:
      t = ILI9341_MADCTL_MV | ILI9341_MADCTL_BGR;
      break;
    case 0:
      t = ILI9341_MADCTL_MY | ILI9341_MADCTL_BGR;
      break;
    case 1:
      t = ILI9341_MADCTL_MX | ILI9341_MADCTL_MY | ILI9341_MADCTL_MV | ILI9341_MADCTL_BGR;
      break;
  }
   writeRegister8(ILI9341_MADCTL, t ); // MADCTL
   // For 9341, init default full-screen address window:
   setAddrWindow(0, 0, _width - 1, _height - 1); // CS_IDLE happens here
  }

#ifdef read8isFunctionalized
  #define read8(x) x=read8fn()
#endif

// Because this function is used infrequently, it configures the ports for
// the read operation, reads the data, then restores the ports to the write
// configuration.  Write operations happen a LOT, so it's advantageous to
// leave the ports in that state as a default.
uint16_t TFTTouchShield::readPixel(int16_t x, int16_t y) {
  return 0;
}

// Ditto with the read/write port directions, as above.
uint16_t TFTTouchShield::readID(void) {

  uint8_t hi, lo;

  if (readReg(0x04) == 0x8000) { // eh close enough
    writeRegister24(HX8357D_SETC, 0xFF8357);
    delay(300);
    //Serial.println(readReg(0xD0), HEX);
    if (readReg(0xD0) == 0x990000) {
      return 0x8357;
    }
  }

  uint16_t id = readReg(0xD3);
  if (id == 0x9341) {
    return id;
  }

  CS_ACTIVE;
  CD_COMMAND;
  write8(0x00);
  WR_STROBE;     // Repeat prior byte (0x00)
  setReadDir();  // Set up LCD data port(s) for READ operations
  CD_DATA;
  read8(hi);
  read8(lo);
  setWriteDir();  // Restore LCD data port(s) to WRITE configuration
  CS_IDLE;

  id = hi; id <<= 8; id |= lo;
  return id;
}

uint32_t TFTTouchShield::readReg(uint8_t r) {
  uint32_t id;
  uint8_t x;

  // try reading register #4
  CS_ACTIVE;
  CD_COMMAND;
  write8(r);
  setReadDir();  // Set up LCD data port(s) for READ operations
  CD_DATA;
  delayMicroseconds(50);
  read8(x);
  id = x;          // Do not merge or otherwise simplify
  id <<= 8;              // these lines.  It's an unfortunate
  read8(x);
  id  |= x;        // shenanigans that are going on.
  id <<= 8;              // these lines.  It's an unfortunate
  read8(x);
  id  |= x;        // shenanigans that are going on.
  id <<= 8;              // these lines.  It's an unfortunate
  read8(x);
  id  |= x;        // shenanigans that are going on.
  CS_IDLE;
  setWriteDir();  // Restore LCD data port(s) to WRITE configuration

  //Serial.print("Read $"); Serial.print(r, HEX); 
  //Serial.print(":\t0x"); Serial.println(id, HEX);
  return id;
}

// Pass 8-bit (each) R,G,B, get back 16-bit packed color
inline uint16_t TFTTouchShield::color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

// For I/O macros that were left undefined, declare function
// versions that reference the inline macros just once:

#ifndef write8
void TFTTouchShield::write8(uint8_t value) {
  write8inline(value);
}
#endif

#ifdef read8isFunctionalized
uint8_t TFTTouchShield::read8fn(void) {
  uint8_t result;
  read8inline(result);
  return result;
}
#endif

#ifndef setWriteDir
void TFTTouchShield::setWriteDir(void) {
  setWriteDirInline();
}
#endif

#ifndef setReadDir
void TFTTouchShield::setReadDir(void) {
  setReadDirInline();
}
#endif

#ifndef writeRegister8
void TFTTouchShield::writeRegister8(uint8_t a, uint8_t d) {
  writeRegister8inline(a, d);
}
#endif

#ifndef writeRegister16
void TFTTouchShield::writeRegister16(uint16_t a, uint16_t d) {
  writeRegister16inline(a, d);
}
#endif

#ifndef writeRegisterPair
void TFTTouchShield::writeRegisterPair(uint8_t aH, uint8_t aL, uint16_t d) {
  writeRegisterPairInline(aH, aL, d);
}
#endif


void TFTTouchShield::writeRegister24(uint8_t r, uint32_t d) {
  CS_ACTIVE;
  CD_COMMAND;
  write8(r);
  CD_DATA;
  delayMicroseconds(10);
  write8(d >> 16);
  delayMicroseconds(10);
  write8(d >> 8);
  delayMicroseconds(10);
  write8(d);
  CS_IDLE;
}

void TFTTouchShield::writeRegister32(uint8_t r, uint32_t d) {
  CS_ACTIVE;
  CD_COMMAND;
  write8(r);
  CD_DATA;
  delayMicroseconds(10);
  write8(d >> 24);
  delayMicroseconds(10);
  write8(d >> 16);
  delayMicroseconds(10);
  write8(d >> 8);
  delayMicroseconds(10);
  write8(d);
  CS_IDLE;
}

TouchPoint::TouchPoint(void) {
    x = y = 0;
}

TouchPoint::TouchPoint(int x0, int y0, int z0)
{
    x = x0;
    y = y0;
    z = z0;
}

bool TouchPoint::operator==(TouchPoint p1)
{
    return  ((p1.x == x) && (p1.y == y) && (p1.z == z));
}

bool TouchPoint::operator!=(TouchPoint p1)
{
    return  ((p1.x != x) || (p1.y != y) || (p1.z != z));
}

bool TouchPoint::isTouching(void)
{
    return (z > __PRESSURE);
}

void TFTTouchShield::setupTouchScreen(uint8_t xp, uint8_t yp, uint8_t xm, uint8_t ym) {
    _yp = yp;
    _xm = xm;
    _ym = ym;
    _xp = xp;
}

#if AVERAGE
#define AVERAGETIME 4
int avr_analog(int adpin)
{
    int sum = 0;
    int max = 0;
    int min = 1024;
    for(int i = 0; i<AVERAGETIME; i++)
    {
        int tmp = analogRead(adpin);
        if(tmp > max)max = tmp;
        if(tmp < min)min = tmp;
        sum += tmp;
        //   sum+=analogRead(adpin);
    }
    return (sum-min-max)/(AVERAGETIME-2);

}
#endif

TouchPoint TFTTouchShield::getPoint(void) {
    int x, y, z = 1;
    int samples[NUMSAMPLES];
    uint8_t i, valid;

    uint8_t xp_port = digitalPinToPort(_xp);
    unsigned char yp_port = digitalPinToPort(_yp);
    unsigned char xm_port = digitalPinToPort(_xm);
    unsigned char ym_port = digitalPinToPort(_ym);

    unsigned char xp_pin = digitalPinToBitMask(_xp);
    unsigned char yp_pin = digitalPinToBitMask(_yp);
    unsigned char xm_pin = digitalPinToBitMask(_xm);
    unsigned char ym_pin = digitalPinToBitMask(_ym);
    valid = 1;
    pinMode(_yp, INPUT);
    pinMode(_ym, INPUT);

    *portOutputRegister(yp_port) &= ~yp_pin;
    *portOutputRegister(ym_port) &= ~ym_pin;

    pinMode(_xp, OUTPUT);
    pinMode(_xm, OUTPUT);

    *portOutputRegister(xp_port) |= xp_pin;
    *portOutputRegister(xm_port) &= ~xm_pin;

    for (i=0; i<NUMSAMPLES; i++)
    {
#if AVERAGE
        samples[i] = avr_analog(_yp);
#else
        samples[i] = analogRead(_yp);
#endif
    }

#if !COMP
    if (samples[0] != samples[1]) { valid = 0; }
#else
    int icomp = samples[0]>samples[1]?samples[0]-samples[1]:samples[1] - samples[0];
    if(icomp > COMP)valid = 0;
#endif

    x = (samples[0] + samples[1]);

    pinMode(_xp, INPUT);
    pinMode(_xm, INPUT);
    *portOutputRegister(xp_port) &= ~xp_pin;

    pinMode(_yp, OUTPUT);
    *portOutputRegister(yp_port) |= yp_pin;
    pinMode(_ym, OUTPUT);

    for (i=0; i<NUMSAMPLES; i++) {
#if AVERAGE
        samples[i] = avr_analog(_xm);
#else
        samples[i] = analogRead(_xm);
#endif
    }

#if !COMP
    if (samples[0] != samples[1]) { valid = 0; }
#else
    icomp = samples[0]>samples[1]?samples[0]-samples[1]:samples[1] - samples[0];
    if(icomp>COMP)valid = 0;
#endif
    y = (samples[0]+samples[0]);

    pinMode(_xp, OUTPUT);
    *portOutputRegister(xp_port) &= ~xp_pin;            // Set X+ to ground
    *portOutputRegister(ym_port) |=  ym_pin;            // Set Y- to VCC
    *portOutputRegister(yp_port) &= ~yp_pin;            // Hi-Z X- and Y+
    pinMode(_yp, INPUT);

    int z1          = analogRead(_xm);
    int z2          = analogRead(_yp);
    float rtouch    = 0;

    rtouch  = z2;
    rtouch /= z1;
    rtouch -= 1;
    rtouch *= (2046-x)/2;
    rtouch *= RXPLATE;
    rtouch /= 1024;
    z = rtouch;
    if (! valid) {
        z = 0;
    }

    pinMode(_xm, OUTPUT);
    pinMode(_xp, OUTPUT);
    pinMode(_ym, OUTPUT);
    pinMode(_yp, OUTPUT);
    return TouchPoint(x, y, z);
}

bool TFTTouchShield::isTouching(void)
{
    TouchPoint p = getPoint();
    return (p.z > __PRESSURE);
}

