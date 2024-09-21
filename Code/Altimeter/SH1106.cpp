#include "Arduino.h"
#include "SH1106.h"
#include "SH1106_font.h"

// Constructor
SH1106::SH1106(uint8_t clk, uint8_t mos, uint8_t res, uint8_t dc, uint8_t cs) {
  clkPin = clk;
  mosPin = mos;
  resPin = res;
  dcPin = dc;
  csPin = cs;

  dataMode = false;
  initialized = false;

  graphicsBuffer = nullptr;
  clearDisplayRamOnNextDisplay = true;
} 

void SH1106::initPins() {
  pinMode(resPin, OUTPUT); 
  digitalWrite(resPin, HIGH);
  pinMode(clkPin, OUTPUT);
  pinMode(mosPin, OUTPUT);
  pinMode(dcPin, OUTPUT);
  pinMode(csPin, OUTPUT);
}

void SH1106::reset() {
  // hold res pin LOW for a while 
  digitalWrite(resPin, LOW); 
  delayMicroseconds(10);
  digitalWrite(resPin, HIGH); //delayMicroseconds(10);
}

void SH1106::TransferStart() {
  digitalWrite(csPin, LOW);  // CS (Upper score) 
}

void SH1106::TransferEnd() {
  digitalWrite(csPin, HIGH);
}

void SH1106::CommandMode() {
  dataMode = false; // =0
  digitalWrite(dcPin, LOW); // A0 set to LOW
}

void SH1106::DataMode() {
  dataMode = true; // =1
  digitalWrite(dcPin, HIGH); // A0 set to HIGH
}

// bit-bashed SPI write a byte code using cls and mos pins
void SH1106::Write(uint8_t d) {
  for (int i=7; i>=0; --i) {
    digitalWrite(clkPin, LOW);
    digitalWrite(mosPin, d >> i & 1);
    digitalWrite(clkPin, HIGH);
  }
}

void SH1106::init() {
  if (!initialized) { 
    initPins();
    reset();
    clearDisplayRam();
    configureDefault();
    initialized = true;
  }
}

void SH1106::clearDisplayRam() {
  TransferStart();
  CommandMode();
  Write(SH1106_SET_COLUMN_ADDR_LOW | 2);
  for (uint8_t page = 0; page < 8; page++) {
    CommandMode();
    Write(SH1106_SET_PAGE_ADDR | page);
    Write(SH1106_SET_COLUMN_ADDR_HIGH | 0);
    DataMode();
    for (uint8_t i = 0; i < 128; i++) Write(0);
  }
  TransferEnd();
  clearDisplayRamOnNextDisplay = false;
}

void SH1106::configureDefault() {
  TransferStart();
  CommandMode();
  Write(SH1106_SET_SEGMENT_REMAP | 1);  // flip horizontally
  Write(SH1106_SET_COM_SCAN_DIR | 8);   // flip vertically
  Write(SH1106_SET_CONTRAST);
  Write(0xFF);                          // maximum brightness
  Write(SH1106_SET_DISPLAY_ON | 1);
  TransferEnd();
}

void SH1106::invert() {
  init();
  TransferStart();
  CommandMode();
  Write(SH1106_SET_INVERT_DISPLAY | 1);
  TransferEnd();
}

void SH1106::rotate180() {
  init();
  TransferStart();
  CommandMode();
  Write(SH1106_SET_SEGMENT_REMAP);
  Write(SH1106_SET_COM_SCAN_DIR);
  TransferEnd();
}

void SH1106::display8x2Text() {
  TransferStart();
  writeSegmentUpperText(2, 2 + 17, getLinePointer(0), 8);
  writeSegmentLowerText(3, 2 + 17, getLinePointer(0), 8);
  writeSegmentUpperText(5, 2 + 17, getLinePointer(1), 8);
  writeSegmentLowerText(6, 2 + 17, getLinePointer(1), 8);
  TransferEnd();
}

void SH1106::display8x2TextPartial(uint8_t x, uint8_t y, uint8_t width) {
    if (x >= 8 || y >= 2) { return; }
    if (width > (uint8_t)(8 - x)) { width = 8 - x; }
    if (width == 0) { return; }

    const uint8_t page = 2 + y * 3;
    const uint8_t columnAddr = 2 + 17 + x * 12;
    const uint8_t * const text = getLinePointer(y) + x;

    TransferStart();
    writeSegmentUpperText(page, columnAddr, text, width);
    writeSegmentLowerText(page + 1, columnAddr, text, width);
    TransferEnd();
  }

void SH1106::displayPartial(uint8_t x, uint8_t y, uint8_t width) {
  init();
  if (clearDisplayRamOnNextDisplay) { clearDisplayRam(); }
  display8x2TextPartial(x, y, width);
}

void SH1106::writeSegmentUpperText(uint8_t page, uint8_t columnAddr, const uint8_t * text, uint8_t textLength) {
  CommandMode();
  Write(SH1106_SET_PAGE_ADDR | page);
  Write(SH1106_SET_COLUMN_ADDR_HIGH | (columnAddr >> 4));
  Write(SH1106_SET_COLUMN_ADDR_LOW | (columnAddr & 0xF));
  DataMode();
  for (uint8_t i = 0; i < textLength; i++) {
    uint8_t glyph = *text++;
    for (uint8_t pixelX = 0; pixelX < 5; pixelX++) {
      uint8_t column = repeatBits(getGlyphColumn(glyph, pixelX) & 0xF);
//          column = 0xFF;
      Write(column); Write(column);
    }
    Write(0); Write(0);
  }
}

void SH1106::writeSegmentLowerText(uint8_t page, uint8_t columnAddr, const uint8_t * text, uint8_t textLength) {
    CommandMode();
    Write(SH1106_SET_PAGE_ADDR | page);
    Write(SH1106_SET_COLUMN_ADDR_HIGH | (columnAddr >> 4));
    Write(SH1106_SET_COLUMN_ADDR_LOW | (columnAddr & 0xF));
    DataMode();
    for (uint8_t i = 0; i < textLength; i++) {
      uint8_t glyph = *text++;
      for (uint8_t pixelX = 0; pixelX < 5; pixelX++) {
        uint8_t column = repeatBits(getGlyphColumn(glyph, pixelX) >> 4);
//        column = 0xFF;
        Write(column); Write(column);
      }
      Write(0); Write(0);
    }
  }

uint8_t SH1106::getGlyphColumn(uint8_t glyph, uint8_t pixelX) {
  if (glyph >= 0x20) {
    return pgm_read_byte(&Font[glyph - 0x20][pixelX]);
  }
  /*
  else if (glyph < 8) {
    return customChars[glyph][pixelX];
  }
  */
  else {
    return 0;
  }
}

uint8_t * SH1106::getLinePointer(uint8_t line) {
  return textBuffer + line * textBufferWidth;
}

void SH1106::gotoXY(uint8_t x, uint8_t y) {
  textCursorX = x;
  textCursorY = y;
}

// displays a block of data to screen, at top-left-hand position (page, col),
// ie pixel row is pagex8 and pixel column is col.
// the height is pages high (ie pagesx8 pixels) and cols pixels wide.
// the data (in vertical-encoding) is stored in a 1d array of bytes called array,
// while the offset for individual bytes (to display) is in address which has to be uint16_t
// since de array might have more than 256 elements.
void SH1106::writeBlock(uint8_t page, uint8_t col, uint8_t pages, uint8_t cols, uint16_t address) {
  uint8_t columnByte;
  TransferStart();
  CommandMode();
  Write(SH1106_SET_PAGE_ADDR | page);
  Write(SH1106_SET_COLUMN_ADDR_HIGH | ((col + 2) >> 4));
  Write(SH1106_SET_COLUMN_ADDR_LOW | ((col + 2) & 0xF));
  DataMode();
  for (uint8_t i = 0; i < cols; i++) {
    columnByte = pgm_read_byte(&Blob[address + i]); 
    Write(columnByte);
  }  
  TransferEnd();
}

// displays an 8x8 char at specified page and column position,
// with char data bytes x8 (in vertical encoding) being obtained from Font2[][8] array
// charCode can be input as eg 'A' since they are implicitly uint8_t.
// need a mysterious 2 pixel offset for column !?
void SH1106::write8x8Char(uint8_t page, uint8_t column, uint16_t charCode) {
  uint8_t columnByte;
  TransferStart();
  CommandMode();
  Write(SH1106_SET_PAGE_ADDR | page);
  Write(SH1106_SET_COLUMN_ADDR_HIGH | ((column+2) >> 4));
  Write(SH1106_SET_COLUMN_ADDR_LOW | ((column+2) & 0xF));
  DataMode();
  for (uint8_t i = 0; i < 8; i++) {
    columnByte = pgm_read_byte(&Font8x8[charCode][i]); 
    Write(columnByte);
  }  
  TransferEnd();
}




// After writing to screen need to call this to stop cursor?
void SH1106::writeEND() {
  TransferStart();
  CommandMode();
  Write(SH1106_SET_PAGE_ADDR | 0);
  Write(SH1106_SET_COLUMN_ADDR_HIGH | 0);
  Write(SH1106_SET_COLUMN_ADDR_LOW | 0);
  DataMode();
  Write(0);
  TransferEnd();
}

// The only function that is not in the SH1106 class
static inline uint8_t repeatBits(uint8_t d) {
  return pgm_read_byte(repeatBitsTable + d);
}

