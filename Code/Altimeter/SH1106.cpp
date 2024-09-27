#include "Arduino.h"
#include "SH1106.h"
#include "Display.h"

// Constructor
SH1106::SH1106(uint8_t clk, uint8_t mos, uint8_t res, uint8_t dc, uint8_t cs) {
  // set pin numbers
  clkPin = clk; mosPin = mos; resPin = res; dcPin = dc; csPin = cs;
  init();

    // initialise str_old*, for its first use in loop()
  for (int i=0; i<16; i++) {
    str_old4[i] = ' ';
    str_old3[i] = ' ';
    str_old2[i] = ' ';
    str_old1[i] = ' ';
    str_old0[i] = ' ';
    str_old[i] = ' ';
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

// generate and display formatted string for altitude,
// 3 pages (24 bits) high, In feet, 1 dp, can change position.
// but for speed only redisplay changed characters.
void SH1106::Altitude_smallfont(float altitude, uint8_t page, uint8_t col) {
  dtostrf(altitude,8,1,str_new);
  if (altitude >= 1000.0) {
    // add a comma to seperate thousands eg 14,234.3
    str_new[0]=str_new[1];
    str_new[1]=str_new[2];
    str_new[2]=','; 
  }
  for (int i=0; i<8; i++) {
    uint16_t charOfs;
    if (str_new[i] != str_old[i]) {
      writeBlock(page, col+16*i, 3, 16,  ASCII2offset(str_new[i], 0x0030), FontNums16x24_);
    }
    str_old[i] = str_new[i]; // after loop finish make str_old the current str_new
  }
}     


// generate and display formatted string for altitude,
// 6 pages (48 bits) high, in thousands of whole feet, fixed position.
// but for speed only redisplay changed characters.
void SH1106::Altitude_largefont(float altitude) {
  bool neg = (altitude < 0);
  if (neg) altitude = -1.0*altitude;
  dtostrf(altitude,5,0,str_new4);
  if (altitude < 10000.0) str_new4[0] = ' ';
  if (altitude < 1000.0) str_new4[1] = '0';
  if (altitude < 100.0) str_new4[2] = '0';
  if (altitude < 10.0) str_new4[3] = '0';

  // display first 3 digits of altitude (eg 134 of 13456) in biggest font
  if (str_new4[0] != str_old4[0]) writeBlock(2, 32*0, 6, 32, ASCII2offset(str_new4[0], 0x00C0), FontNums32x48_);
  if (str_new4[1] != str_old4[1]) writeBlock(2, 32*1, 6, 32, ASCII2offset(str_new4[1], 0x00C0), FontNums32x48_);
  if (str_new4[2] != str_old4[2]) writeBlock(2, 32*2, 6, 32, ASCII2offset(str_new4[2], 0x00C0), FontNums32x48_);

  // display last 2 digits of altitude (eg 56 of 13456) in small font
  if (str_new4[3] != str_old4[3]) writeBlock(5, 32*3, 3, 16, ASCII2offset(str_new4[3], 0x0030), FontNums16x24_);
  if (str_new4[4] != str_old4[4]) writeBlock(5, 32*3+16, 3, 16, ASCII2offset(str_new4[4], 0x0030), FontNums16x24_);
}



// a private utility function that maps numbers 'only' font chars to memmory offsets
// in font bitmap arrays, needs offsetScale argument to make if useful for different size fonts
// eg. FontNums32x48_ needs offsetScale=192, while FontNums16x24_ needs offsetScale=48
uint16_t SH1106::ASCII2offset(char char_, uint16_t offsetScale) {
  uint16_t charOfs;
  switch (char_) {
    case '0': charOfs = 0; break;
    case '1': charOfs = 1; break;
    case '2': charOfs = 2; break;
    case '3': charOfs = 3; break;
    case '4': charOfs = 4; break;
    case '5': charOfs = 5; break;
    case '6': charOfs = 6; break;
    case '7': charOfs = 7; break;
    case '8': charOfs = 8; break;
    case '9': charOfs = 9; break;
    case ' ': charOfs = 10; break;
    case '-': charOfs = 11; break;
    case ',': charOfs = 12; break;
    case '.': charOfs = 13; break;
    default: charOfs = 11;
  };
  return charOfs*offsetScale;
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

