#include <Arduino.h>

#define SH1106_SET_COLUMN_ADDR_LOW 0x00
#define SH1106_SET_COLUMN_ADDR_HIGH 0x10
#define SH1106_SET_CONTRAST 0x81
#define SH1106_SET_SEGMENT_REMAP 0xA0
#define SH1106_SET_INVERT_DISPLAY 0xA6
#define SH1106_SET_DISPLAY_ON 0xAE
#define SH1106_SET_PAGE_ADDR 0xB0
#define SH1106_SET_COM_SCAN_DIR 0xC0

static inline uint8_t repeatBits(uint8_t d);

class SH1106 {

  public:
/*  
 The module (in 4-wire SPI mode) has 7 pins (which on the Arduino PRO Mini with HW SPI use):
 1 GND - connect to ground pin on micro (Arduino Pro Mini 5V 16Mhz assumed here)
 2 VCC - connect to VCC pin on micro (assumed 5V)
 3 CLK - Clock, connect to Pin 13 on micro (assumed by default) - WIRE 1 
       - Called SCL Wire 3 in SH1106 pdf
  MISO - Master In Slave Out, NOT AVAILABLE HERE - but would be pin 12 on micro - WIRE 3 
 4 MOSI - Master Out Slave In, connect to pin 11 on micro (assumed by default) - WIRE 2
        - also called SDI or SI Wire 2 in SH1106 pdf
 5 RES - Reset, connect to Pin 8 (can change) 
       - Called RES (Upper score) in SH1106 pdf
 6 DC - Data/Command, connect to pin 9 (can change)
      - Called A0 (HIGH= data mode, LOW= command mode) Wire 4 in SH1106 pdf 
 7 CS - Chip Select, connect to pin 10 (can change) - WIRE 4 
      - Called CS (upper score) Wire 1 in SH1106 pdf
*/
  SH1106(uint8_t clk, uint8_t mos, uint8_t res, uint8_t dc, uint8_t cs); // Constructor

  private:
  
  uint8_t clkPin, mosPin, resPin, dcPin, csPin;
  bool initialized;
  bool dataMode;
  bool clearDisplayRamOnNextDisplay;
  bool disableAutoDisplay;
  static const uint8_t textBufferWidth = 21, textBufferHeight = 8;
  uint8_t textBuffer[textBufferHeight * textBufferWidth];
  uint8_t textCursorX;
  uint8_t textCursorY;
  const uint8_t * graphicsBuffer;

  // low level SPI comms with SH1106
  void initPins();
  void reset();
  void TransferStart();
  void TransferEnd();
  void CommandMode();
  void DataMode();
  void Write(uint8_t d);
  void init();
  void clearDisplayRam();
  void configureDefault();

  void writeSegmentUpperText(uint8_t page, uint8_t columnAddr, const uint8_t * text, uint8_t textLength);
  void writeSegmentLowerText(uint8_t page, uint8_t columnAddr, const uint8_t * text, uint8_t textLength);
  uint8_t getGlyphColumn(uint8_t glyph, uint8_t pixelX);
  uint8_t * getLinePointer(uint8_t line);

  void display8x2Text(); // = displayFunction
  void display8x2TextPartial(uint8_t x, uint8_t y, uint8_t width); // = displayPartialFunction 
  void displayPartial(uint8_t x, uint8_t y, uint8_t width);
  

  public:

  void invert();
  void rotate180();
  void gotoXY(uint8_t x, uint8_t y);
  size_t write(uint8_t d);
  void writeBlock(uint8_t page, uint8_t columnAddr, uint8_t columnNum, uint8_t column);
  void writeEND();
};

