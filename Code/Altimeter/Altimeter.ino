#include "MS5637.h"
#include "SH1106.h"
#include "Display.h"
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

// An instance of the BaroSensorClass called Baro is created
// The module is I2C and has just 4 pins:
// 1 GND - connect to ground pin on micro (Arduino Pro Mini 5V 16Mhz assumed here)
// 2 SDA - connect to SDA pin on micro (also called D2)
// 3 SCL - connect to SCL pin on micro (also called D3)
// 4 VCC - connect to VCC pin on micro (assumed 5V)
BaroSensorClass Baro;

// Pin connected to button, other side of button connected to GND
// Pin set to INPUT_PULLUP (to not require external resistor)
const int buttonPin = A0; // Pin connected to the push button (14)

// Pin connected to red LED Anode (long+), LED Cathode(short-) connected to 220 R resistor,
// other side of resistor connected to GND
const int ledPin = A1; // LED pin (15) 

float temp, pressure, altBase, alt, altRel, cellPer, cellVol;
char str_old2[17];
char str_new2[17];
char str_old1[17];
char str_new1[17];
char str_old0[17];
char str_new0[17];
char str_old[17];
char str_new[17];

void setup()
{
  Serial.begin(9600);
  while (!Serial) delay (10);

  Serial.println(F("\nAdafruit MAX17048 simple demo"));
  if (!lipo.begin()) {
    Serial.println(F("Couldn't find Adafruit MAX17048? Make sure a battery is plugged in!"));
    while (1) delay(10);
  }
  Serial.print(F("Found MAX17048 with Chip ID: 0x"));
  Serial.println(lipo.getChipID(), HEX);


  pinMode(buttonPin, INPUT_PULLUP); // set the button pin as input with internal pull-up resistor
  pinMode(ledPin, OUTPUT); // Set the LED pin as output

  // setup MS5637 sensor (An instance of the BaroSensorClass object BaroSensor has been constructed above)
  Baro.begin();
  Baro.dumpDebugOutput();
  Baro.getTempAndPressure(&temp, &pressure);
  //altBase = BaroSensor.pressure2altitude(pressure);
  altBase = 0;
  Serial.println(altBase);

  // initialise str_old*, for its first use in loop()
  for (int i=0; i<8; i++) {
    str_old2[i] = ' ';
    str_old1[i] = ' ';
    str_old0[i] = ' ';
    str_old[i] = ' ';
  }  
}


void loop()
{
  cellVol = lipo.cellVoltage();
  if (isnan(cellVol)) {
    Serial.println("Failed to read cell voltage, check battery is connected!");
    delay(2000);
    return;
  }

  cellPer = lipo.cellPercent();
  Serial.print(F("Batt Voltage: "));
  Serial.print(cellVol, 3);
  Serial.println(" V");
  Serial.print(F("Batt Percent: "));
  Serial.print(cellPer, 1);
  Serial.println(" %");
  Serial.println();

  // generate and display formatted string for Battery Voltage cellVol,
  // but for speed only redisplay changed characters.
  dtostrf(cellVol,4,2,str_new1);
  for (int i=0; i<4; i++) {
    if (str_new1[i] != str_old1[i]) {
      OLED.write8x8Char(0, (i+6)*8, str_new1[i], Font8x8);
    }  
    str_old1[i] = str_new1[i]; // after loop finish make str_old1 the current str_new1
  }

  // generate and display formatted string for Battery Percentage cellPer,
  // but for speed only redisplay changed characters.
  dtostrf(cellPer,5,1,str_new2);
  for (int i=0; i<5; i++) {
    if (str_new2[i] != str_old2[i]) {
      OLED.write8x8Char(0, (i+11)*8, str_new2[i], Font8x8);
    }  
    str_old2[i] = str_new2[i]; // after loop finish make str_old1 the current str_new1
  }


  // read the pushButton pin:
  int buttonState = digitalRead(buttonPin);

  if (!Baro.isOK()) {
    // Try to reinitialise the sensor if we can
    Baro.begin();

    // measure temperature and pressure
    Baro.getTempAndPressure(&temp, &pressure);
  }
  else if (buttonState==LOW) { // button is pressed
    // Turn on the LED
    digitalWrite(ledPin, HIGH);
    Serial.println("Button pressed!");

    // measure temperature and pressure and zero the altimeter
    Baro.getTempAndPressure(&temp, &pressure);
    altBase = Baro.pressure2altitude(pressure);
  }
  else { // button is not pressed
    // Turn off the LED
    digitalWrite(ledPin, LOW);

    // measure temperature and pressure
    Baro.getTempAndPressure(&temp, &pressure);
  }

  // Calculates the altitude after zeroing
  alt = Baro.pressure2altitude(pressure);
  altRel = alt - altBase;

  // generate and display formatted string for temp,
  // but for speed only redisplay changed characters.
  dtostrf(temp,5,1,str_new0);
  for (int i=0; i<5; i++) {
    if (str_new0[i] != str_old0[i]) {
      OLED.write8x8Char(0, i*8, str_new0[i], Font8x8);
    }  
    str_old0[i] = str_new0[i]; // after loop finish make str_old0 the current str_new0
  }

  // generate and display formatted string for altRel,
  // but for speed only redisplay changed characters.
  dtostrf(altRel,8,1,str_new);
  if (altRel >= 1000.0 ) {
    // add a comma to seperate thousands eg 14,234.3
    str_new[0]=str_new[1];
    str_new[1]=str_new[2];
    str_new[2]=','; 
  }
  for (int i=0; i<8; i++) {
    uint16_t charOfs;
    if (str_new[i] != str_old[i]) {
      switch (str_new[i]) {
        case '0': charOfs = 0x0000; break;
        case '1': charOfs = 0x0030; break;
        case '2': charOfs = 0x0060; break;
        case '3': charOfs = 0x0090; break;
        case '4': charOfs = 0x00C0; break;
        case '5': charOfs = 0x00F0; break;
        case '6': charOfs = 0x0120; break;
        case '7': charOfs = 0x0150; break;
        case '8': charOfs = 0x0180; break;
        case '9': charOfs = 0x01B0; break;
        case '-': charOfs = 0x01E0; break;
        case ',': charOfs = 0x0210; break;
        case '.': charOfs = 0x0240; break;
        case ' ': charOfs = 0x0270; break;
        default: charOfs = 0x0018;
      }  
      OLED.writeBlock(2, 16*i, 3, 16, charOfs, FontNums16x24);
    }
    str_old[i] = str_new[i]; // after loop finish make str_old the current str_new
  }   



  OLED.writeEND();

  Serial.print(temp);
  Serial.print(" ");
  Serial.print(alt);
  Serial.print(" ");
  Serial.println(altRel);

  delay(2000);
}


