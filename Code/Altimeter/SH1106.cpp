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
    str_old0[i] = ' ';
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
//  Write(0xFF);                          // maximum brightness
  Write(0x10);                          // set lower brightness
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



// generate and display formatted string for Battery Voltage cellVol,
// but for speed only redisplay changed characters.
void SH1106::BatteryVoltage(float cellVol_, uint8_t page, uint8_t col) {
  dtostrf(cellVol_,3,1,str_new1); str_new1[3] = 'v';
  for (int i=0; i<4; i++) {
    if (str_new1[i] != str_old1[i]) write8x8Char(page, col+i*8, str_new1[i], Font8x8_);
    str_old1[i] = str_new1[i]; // after loop finish make str_old1 the current str_new1
  }
}

// generate and display formatted string for Battery Percentage cellPer,
// but for speed only redisplay changed characters.
void SH1106::BatteryPercentage(float cellPer_, uint8_t page, uint8_t col) {
  /*
  dtostrf(cellPer_,5,1,str_new2); str_new2[5] = '%';
  for (int i=0; i<6; i++) {
    if (str_new2[i] != str_old2[i]) write8x8Char(page, col+i*8, str_new2[i], Font8x8_);
    str_old2[i] = str_new2[i]; // after loop finish make str_old1 the current str_new1
  }
  */
  dtostrf(cellPer_,2,0,str_new2);
  str_new2[2] = 1; // points to custom small percent sign 
  if (cellPer_ >= 99.5) {
    // the text FULL in 3 custom 8x8 characters (fudge!)
    str_new2[0] = 2;
    str_new2[1] = 3;
    str_new2[2] = 4;
  }
  for (int i=0; i<3; i++) {
    if (str_new2[i] != str_old2[i]) write8x8Char(page, col+i*8, str_new2[i], Font8x8_);
    str_old2[i] = str_new2[i]; // after loop finish make str_old1 the current str_new1
  }
}  

// generate and display a graphic for Battery level, with charging differentiation
// for speed only redisplay changed bytes
void SH1106::BatteryLevelGraphic(float cellPer_, uint8_t page, uint8_t col, bool chargeing) {
  // Setup default full battery graphic
  str_new3[0] = 0;
  str_new3[1] = 1;
  for (int i=2; i<11; i++) str_new3[i] = 5;
  if (!chargeing) { 
    str_new3[12] = 1;
    str_new3[13] = 0;
    str_new3[14] = 6;
    str_new3[15] = 6;
  } else { // if chargeing
    str_new3[12] = 11;
    str_new3[13] = 12;
    str_new3[14] = 13;
    str_new3[15] = 14;
  }

  // number of whole 10%'s in cellPer
  if (cellPer_ >= 100.0) cellPer_ = 100.0;
  if (cellPer_ <= 0.0) cellPer_ = 0.0;

  int full_Levels = (int)(cellPer_/10.0);
  float f = cellPer_-(float)full_Levels*10.0;
  if (f > 8.75) ++full_Levels;  
  int blank_Levels = 10-full_Levels; 

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
  // Note: full level bars did not have to be explicitly set since they where initially
  // set in the default.
  for (int i=0; i<16; i++) {
    if (str_new3[i] != str_old3[i]) writeBlock(page, col+i, 1, 1, str_new3[i], Battery_);
    str_old3[i] = str_new3[i]; // after loop finish make str_old1 the current str_new1
  }
}

// generate and display a graphic for a Battery Error,
 // but for speed only redisplay changed bytes
void SH1106::BatteryErrorGraphic(uint8_t page, uint8_t col) {
  str_new3[0] = 0;
  str_new3[1] = 1;
  str_new3[2] = 7;
  str_new3[3] = 7;
  str_new3[4] = 8;
  str_new3[5] = 8;
  str_new3[6] = 9;
  str_new3[7] = 9;
  str_new3[8] = 8;
  str_new3[9] = 8;
  str_new3[10] = 7;
  str_new3[11] = 7;
  str_new3[12] = 1;
  str_new3[13] = 0;
  str_new3[14] = 6;
  str_new3[15] = 6;
  for (int i=0; i<16; i++) writeBlock(page, col+i, 1, 1, str_new3[i], Battery_);
}

// generate and display formatted string for temperature temp_,
// but for speed only redisplay changed characters.
void SH1106::Temperature(float temp_, uint8_t page, uint8_t col) {
  //temp_ = -20.6;
  int w = 5; // width of text in characters
  if (temp_<=-10.0) {
    dtostrf(temp_,3,0,str_new0);  
    str_new0[3] = 0x00; // degree character
    w--;
  } else {
    dtostrf(temp_,4,1,str_new0);
    str_new0[4] = 0x00; // degree character
  }
  for (int i=0; i<w; i++) {
    if (str_new0[i] != str_old0[i]) write8x8Char(page, col+i*8, str_new0[i], Font8x8_);
    str_old0[i] = str_new0[i]; // after loop finish make str_old0 the current str_new0
  }
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

