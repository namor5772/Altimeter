
#include "MS5637.h"
#include "SH1106.h"
#include <PololuSH1106.h>

#define SMALL

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

// The pins are specified in this order: CLK, MOS, RES, DC, CS.
#ifdef SMALL
  // 3-Wire SPIThe pins are specified in this order: CLK, MOS, RES, DC, CS.
  SH1106 OLED(13, 11, 8, 9, 10);
#else  
  PololuSH1106 display(13 /* CLK */, 11 /* MOSI */, 8 /* RES */, 9 /* DC */, 10 /* CS */);
#endif  

// an instance of the BaroSensorClass called Baro is created
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

float temp, pressure, altBase, alt, altRel;
char str1[11];
char str2[11];


void setup()
{
  Serial.begin(9600);

  pinMode(buttonPin, INPUT_PULLUP); // set the button pin as input with internal pull-up resistor
  pinMode(ledPin, OUTPUT); // Set the LED pin as output

  // setup MS5637 sensor (An instance of the BaroSensorClass object BaroSensor has been constructed in MS5637.cpp)
  Baro.begin();
  Baro.dumpDebugOutput();
  Baro.getTempAndPressure(&temp, &pressure);
  //altBase = BaroSensor.pressure2altitude(pressure);
  altBase = 0;
  Serial.println(altBase);
 
#ifdef SMALL
  OLED.rotate180();
#else
  //display.invert();
  display.rotate180();
  display.setContrast(255);
  display.setLayout11x4();
#endif  

  
}


void loop()
{
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

  Serial.print(temp);
  Serial.print(" ");
  Serial.print(alt);
  Serial.print(" ");
  Serial.println(altRel);
  
  // Display the data on the OLED screen

#ifdef SMALL
  uint8_t s = 18;
  int i = 0;
  int d = 100;
  OLED.write8x8Char(0, i, 'H'); i=i+8; delay(d);
  OLED.write8x8Char(0, i, 'e'); i=i+8; delay(d);
  OLED.write8x8Char(0, i, 'l'); i=i+8; delay(d);
  OLED.write8x8Char(0, i, 'l'); i=i+8; delay(d);
  OLED.write8x8Char(0, i, 'o'); i=i+8; delay(d);
  i = 0;
  OLED.write8x8Char(1, i, 'L'); i=i+8; delay(d);
  OLED.write8x8Char(1, i, 'e'); i=i+8; delay(d);
  OLED.write8x8Char(1, i, 'e'); i=i+8; delay(d);
  OLED.write8x8Char(1, i, '-'); i=i+8; delay(d);
  OLED.write8x8Char(1, i, 'A'); i=i+8; delay(d);
  OLED.write8x8Char(1, i, 'n'); i=i+8; delay(d);
  OLED.write8x8Char(1, i, 'n'); i=i+8; delay(d);
  OLED.write8x8Char(1, i, ' '); i=i+8; delay(d);
  OLED.write8x8Char(1, i, '!'); i=i+8; delay(d);
  OLED.write8x8Char(4, 0, 0x80);
  OLED.write8x8Char(4, 10, 0x80);
  OLED.write8x8Char(4, 20, 0x41);
  OLED.write8x8Char(4, 30, 'A');
  OLED.writeBlock(5, 0, 1, 8, 0x0000);
  OLED.writeBlock(5, 8, 1, 8, 0x0008);
  OLED.writeBlock(5, 16, 1, 8, 0x0010);
  OLED.writeBlock(5, 24, 1, 8, 0x0018);
  

  OLED.writeEND();
#else
  display.gotoXY(0, 0);
  strcpy(str1," ft");
  dtostrf(altRel,7,1,str2);
  strcat(str2,str1);
  display.print(str2);
  display.gotoXY(2, 2);
  strcpy(str1," ft");
  dtostrf(alt,5,0,str2);
  strcat(str2,str1);
  display.print(str2);
#endif
  
  delay(200);
}
