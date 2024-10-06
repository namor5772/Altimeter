#include "MS5637.h"
#include "SH1106.h"
#include <Adafruit_MAX1704X.h>

Adafruit_MAX17048 lipo;

/*************************************************************************************************
Programming the Arduino PRO Mini using a duinothech USB to FTDI Serial Adatptor Module (XC-4464),
Note: This assumes the PRO Mini has a valid burnt bootloader.
Pins from top left hand corner of Pro Mini downwards (funny names on board),
also pinout to using an Arduino Nano as FTDI serial adaptor:
1 GRN - connect to   DTR   on FTDI adapter or   RST   on Nano programmer
2 TXO - connect to   RXD   on FTDI adapter or   TXI   on Nano programmer
3 RXI - connect to   TXD   on FTDI adapter or   RXO   on Nano programmer
4 VCC - connect to   5V    on FTDI adapter or   5V    on Nano programmer
5 GND - connect to   CTS   on FTDI adapter or   GND   on Nano programmer
6 BLK - connect to   GND   on FTDI adapter or   GND   on Nano programmer
*************************************************************************************************/

// An instance of the SH1106 class called OLED is created
// The module uses an enhanced 3-Wire SPI communications protcol
// The specified pins are: CLK, MOS, RES, DC, CS.
SH1106 OLED(13, 11, 8, 9, 10);

// An instance of the MS5637 called BARO is created
// The module is I2C and has just 4 pins:
// 1 GND - connect to ground pin on micro (Arduino Pro Mini 5V 16Mhz assumed here)
// 2 SDA - connect to SDA pin on micro (also called D2)
// 3 SCL - connect to SCL pin on micro (also called D3)
// 4 VCC - connect to VCC pin on micro (assumed 5V)
MS5637 BARO;

// Pin connected to push button (14) , other side of button connected to GND
// Pin set to INPUT_PULLUP (to not require external resistor)
//const int buttonPin = A0;

// Pin (15) connected to red LED Anode (long+) , LED Cathode(short-) connected to 220 R resistor,
// other side of resistor connected to GND
//const int ledPin = A1;

 // Pin connected to the 5V level of the lipo charging board (16)
const int chargePin = A2;

// global variables
float temp, pressure, altBase, altRel, cellPer, cellVol;
float test = 0.0;

void setup() {
  Serial.begin(9600);
  
  OLED.setBrightness(0xFF);
  while(!lipo.begin()) {
    Serial.println(F("Couldn't find Adafruit MAX17048? Make sure a battery is plugged in!"));
    OLED.BatteryErrorGraphic(0, 112);
    delay(5000);
  }
  Serial.print(F("Found MAX17048 with Chip ID: 0x"));
  Serial.println(lipo.getChipID(), HEX);

// pinMode(buttonPin, INPUT_PULLUP); // set the button pin as input with internal pull-up resistor
// pinMode(ledPin, OUTPUT); // Set the LED pin as output
  pinMode(chargePin, INPUT); // set the button pin as input (there are some floating problems?)

  // setup MS5637 sensor (An instance of the MS5637 object BARO has been constructed above)
  BARO.begin();
  BARO.dumpDebugOutput();
  BARO.getTempAndPressure(&temp, &pressure);
  altBase = BARO.pressure2altitude(pressure);
  Serial.println(altBase);
}


void loop() {
  cellVol = lipo.cellVoltage();
  if (isnan(cellVol)) {
    Serial.println("Failed to read cell voltage, check battery is connected!");
    OLED.BatteryErrorGraphic(0, 112);
    delay(2000);
    return;
  }
  cellPer = lipo.cellPercent();
  OLED.BatteryVoltage(cellVol, 0, 48);
  OLED.BatteryPercentage(cellPer, 0, 88);
  //int chargeState = digitalRead(chargePin);
  //bool charging = (chargeState==HIGH)
  OLED.BatteryLevelGraphic(cellPer, 0, 112, (digitalRead(chargePin)==HIGH));

  if (!BARO.isOK()) {
    // Try to reinitialise the sensor if we can and measure temperature and pressure
    BARO.begin();
    BARO.getTempAndPressure(&temp, &pressure);
  }
  else { // just normal measurements on each loop
    BARO.getTempAndPressure(&temp, &pressure);
  }

  // Calculate altitude relative to where the altimeter was turned on
  //alt = BARO.pressure2altitude(pressure);
  altRel = BARO.pressure2altitude(pressure) - altBase;

  // display current temperature measured by the MS5637 (and used to
  // improve calculation of air pressure)
  OLED.Temperature(temp, 0, 0);

  // display current altitudete (adjusted to ground) measured by the MS5637
  OLED.Altitude_largefont(altRel+test);
  // OLED.Altitude_smallfont(altRel, 2, 0);

  OLED.writeEND();

/*
  // TEST SECTION
  if ((test<=13000.0)&&(test>=0.0)) {
    if (test<10) {
      test += 1;
    } else if (test<200) {
      OLED.setBrightness(0xFF);
      test += 7;
    } else if (test<1000) {
      OLED.setBrightness(0xC0);
      test += 17;
    } else if (test<4000) {
      OLED.setBrightness(0xA0);
      test += 29;
    } else if (test<8000) {
      OLED.setBrightness(0x10);
      test += 37;
    } else {
      OLED.setBrightness(0x00);
      test +=47;
    }
  }
  else if (test>13000) {
    OLED.setBrightness(0x20);
    test = -1.0;   
  }
  else if ((test<0)&&(test>=-13000)) {
    if (test>-10) {
      test -= 1;
    } else if (test>-200) {
      OLED.setBrightness(0xFF);
      test -= 7;
    } else if (test>-1000) {
      OLED.setBrightness(0xA0);
      test -= 17;
    } else if (test>-4000) {
      OLED.setBrightness(0x50);
      test -= 29;
    } else if (test>-8000) {
      OLED.setBrightness(0x10);
      test -= 37;
    } else {
      OLED.setBrightness(0x00);
      test -=47;
    }
  }
  else { //if (test<-13000)
    OLED.setBrightness(0x20);
    test = 1.0;
  }
*/  
  OLED.writeEND();

  Serial.print(temp);
  Serial.print(" ");
  Serial.println(altRel);

  delay(100);
}


