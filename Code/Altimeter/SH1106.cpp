#include "Arduino.h"
#include "SH1106.h"
#include "Display2.h"

// Constructor
SH1106::SH1106(uint8_t clk, uint8_t mos, uint8_t res, uint8_t dc, uint8_t cs) {
  // set pin numbers
  clkPin = clk; mosPin = mos; resPin = res; dcPin = dc; csPin = cs;
  init();

    // initialise str_old*, for its first use in loop()
  for (int i=0; i<16; i++) {
    str_old3[i] = ' ';
    str_old2[i] = ' ';
    str_old1[i] = ' ';
//    str_old0[i] = ' ';
//    str_old[i] = ' ';
  }  

} 

void SH1106::TransferStart() {
  digitalWrite(csPin, LOW);  // CS (Upper score) 
}

void SH1106::CommandMode() {
  digitalWrite(dcPin, LOW); // A0 set to LOW
}

void SH1106::DataMode() {
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

void SH1106::TransferEnd() {
  digitalWrite(csPin, HIGH);
}


void SH1106::init() {
    initPins();
    reset();
    clearDisplayRam();
    configureDefault();
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
  digitalWrite(resPin, HIGH);
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
}

void SH1106::configureDefault() {
  TransferStart();
  CommandMode();
  Write(SH1106_SET_SEGMENT_REMAP | 1);  // flip horizontally
  Write(SH1106_SET_COM_SCAN_DIR | 8);   // flip vertically
  Write(SH1106_SET_CONTRAST);
  Write(0xFF);                          // maximum brightness
  Write(SH1106_SET_DISPLAY_ON | 1);

  // rotate 180 - omit if necessary
  Write(SH1106_SET_SEGMENT_REMAP);      
  Write(SH1106_SET_COM_SCAN_DIR);       
  TransferEnd();
}

void SH1106::invert() {
  TransferStart();
  CommandMode();
  Write(SH1106_SET_INVERT_DISPLAY | 1);
  TransferEnd();
}

void SH1106::uninvert() {
  TransferStart();
  CommandMode();
  Write(SH1106_SET_INVERT_DISPLAY | 0);
  TransferEnd();
}


// displays a block of data to screen, at top-left-hand position (page, col),
// ie pixel row is pagex8 and pixel column is col.
// the height is pages high (ie pagesx8 pixels) and cols pixels wide.
// the data (in vertical-encoding) is stored in a 1d array of bytes called Arr,
// while the offset for individual bytes (to display) is address which has to be uint16_t
// since the array might have more than 256 elements.
void SH1106::writeBlock(uint8_t page, uint8_t col, uint8_t pages, uint8_t cols, uint16_t address, const uint8_t Arr[]) {
  uint8_t columnByte;
  TransferStart();
  for (uint8_t j = 0; j < pages; j++) {
    CommandMode();
    Write(SH1106_SET_PAGE_ADDR | (page + j) );
    Write(SH1106_SET_COLUMN_ADDR_HIGH | ((col + 2) >> 4));
    Write(SH1106_SET_COLUMN_ADDR_LOW  | ((col + 2) & 0xF));
    DataMode();
    for (uint8_t i = 0; i < cols; i++) {
      columnByte = pgm_read_byte(&Arr[address + cols*j + i]); 
      Write(columnByte);
    }  
  }
  TransferEnd();
}

// generate and display formatted string for Battery Voltage cellVol,
// but for speed only redisplay changed characters.
void SH1106::BatteryVoltage(float cellVol, uint8_t page, uint8_t col) {
  dtostrf(cellVol,3,1,str_new1); str_new1[3] = 'V';
  for (int i=0; i<4; i++) {
    if (str_new1[i] != str_old1[i]) write8x8Char(page, col+i*8, str_new1[i], Font8x8_);
    str_old1[i] = str_new1[i]; // after loop finish make str_old1 the current str_new1
  }
}

 // generate and display formatted string for Battery Percentage cellPer,
 // but for speed only redisplay changed characters.
void SH1106::BatteryPercentage(float cellPer_, uint8_t page, uint8_t col) {
  dtostrf(cellPer_,3,0,str_new2); str_new2[3] = '%';
  for (int i=0; i<4; i++) {
    if (str_new2[i] != str_old2[i]) write8x8Char(page, col+i*8, str_new2[i], Font8x8_);
    str_old2[i] = str_new2[i]; // after loop finish make str_old1 the current str_new1
  }
}  

 // generate and display a graphic for Battery level,
 // but for speed only redisplay changed bytes
void SH1106::BatteryLevelGraphic(float cellPer_, uint8_t page, uint8_t col) {
  float cp = cellPer_;

  // Setup default full battery graphic
  str_new3[0] = 0;
  str_new3[1] = 1;
  for (int i=2; i<12; i++) str_new3[i] = 5;
  str_new3[12] = 1;
  str_new3[13] = 0;
  str_new3[14] = 6;
  str_new3[15] = 6;

  // number of whole 10%'s in cellPer
  if (cp > 100.0) cp = 100.0;
  if (cp < 0.0) cp = 0.0;

  int full_Levels = (int)(cp/10.0);
  float f = cp-(float)full_Levels*10.0;
  if (f > 8.75) ++full_Levels;  
  int blank_Levels = 10-full_Levels; 

  // determine any full level 10% bars (possibly none)
  for (int i=2; i<=full_Levels+1; i++) str_new3[i] = 5;

  // determine the partial 10% bar if any (there is one or none)
  if ((1.25 < f) && (f <= 8.75)) {
    --blank_Levels;
    if (f < 3.75) {
      str_new3[full_Levels+2] = 2;
    } else if (f < 6.25) {
      str_new3[full_Levels+2] = 3;
    } else if (f <= 8.75) {
      str_new3[full_Levels+2] = 4;
    } else {
      str_new3[full_Levels+2] = 5;
    }
  }

  // determine any empty 10% bars (possibly none)
  for (int i=12-blank_Levels; i<=11; i++) str_new3[i] = 1;

  // display surrounds and all 10 10% bars (as determined above)
  for (int i=0; i<16; i++) {
    if (str_new3[i] != str_old3[i]) writeBlock(page, col+i, 1, 1, str_new3[i], Battery_);
    str_old3[i] = str_new3[i]; // after loop finish make str_old1 the current str_new1
  }
}

// displays an 8x8 char at specified page and column position,
// with char data bytes x8 (in vertical encoding) being obtained from Font2[][8] array
// charCode can be input as eg 'A' since they are implicitly uint8_t.
// need a mysterious 2 pixel offset for column !?
void SH1106::write8x8Char(uint8_t page, uint8_t column, uint16_t charCode, const uint8_t Arr[][8]) {
  uint8_t columnByte;
  TransferStart();
  CommandMode();
  Write(SH1106_SET_PAGE_ADDR | page);
  Write(SH1106_SET_COLUMN_ADDR_HIGH | ((column+2) >> 4));
  Write(SH1106_SET_COLUMN_ADDR_LOW | ((column+2) & 0xF));
  DataMode();
  for (uint8_t i = 0; i < 8; i++) {
    columnByte = pgm_read_byte(&Arr[charCode][i]); 
    Write(columnByte);
  }  
  TransferEnd();
}

// After writing to screen need to call this to stop cursor?
// All hell breaks loose if you dont!
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

