#include <Arduino.h>

#define SH1106_SET_COLUMN_ADDR_LOW 0x00
#define SH1106_SET_COLUMN_ADDR_HIGH 0x10
#define SH1106_SET_CONTRAST 0x81
#define SH1106_SET_SEGMENT_REMAP 0xA0
#define SH1106_SET_INVERT_DISPLAY 0xA6
#define SH1106_SET_DISPLAY_ON 0xAE
#define SH1106_SET_PAGE_ADDR 0xB0
#define SH1106_SET_COM_SCAN_DIR 0xC0

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
Note: The screen pixels are accessed in vertical bytes (least significant bit at top),
8 pages (giving 64 = 8 x 8 bits), there are 128 visible columns, strangely ranging
from 2 to 129.      
*/

class SH1106 {
  public:

  // constructor - includes init()
  SH1106(uint8_t clk, uint8_t mos, uint8_t res, uint8_t dc, uint8_t cs); 

  private:
  
  uint8_t clkPin, mosPin, resPin, dcPin, csPin;

  // specific char arrays used when displaying text on OLED
  char str_old4[17]; char str_new4[17];
  char str_old3[17]; char str_new3[17];
  char str_old2[17]; char str_new2[17];
  char str_old1[17]; char str_new1[17];
  char str_old0[17]; char str_new0[17];
  char str_old[17]; char str_new[17];

  // low level SPI comms with SH1106
  void TransferStart();
  void CommandMode();
  void DataMode();
  void Write(uint8_t d);
  void TransferEnd();

  // setup functions 
  void init(); // just a wrapper for the following functions
  void initPins();
  void reset();
  void clearDisplayRam();
  void configureDefault();

  // specific internal utility functions
  uint16_t ASCII2offset(char char_, uint16_t offsetScale);


  public:

  // core displaying functions
  void invert();
  void uninvert();
  void writeBlock(uint8_t page, uint8_t col, uint8_t pages, uint8_t cols, uint16_t address, const uint8_t Arr[]);
  void write8x8Char(uint8_t page, uint8_t column, uint16_t charCode, const uint8_t Arr[][8]);

  // specific displaying functions
  void BatteryVoltage(float cellVol_, uint8_t page, uint8_t col);
  void BatteryPercentage(float cellPer_, uint8_t page, uint8_t col);
  void BatteryLevelGraphic(float cellPer_, uint8_t page, uint8_t col, bool chargeing);
  void BatteryErrorGraphic(uint8_t page, uint8_t col);
  void Temperature(float temp_, uint8_t page, uint8_t col);
  void Altitude_smallfont(float altitude, uint8_t page, uint8_t col);
  void Altitude_largefont(float altitude);

  void writeEND();
};


