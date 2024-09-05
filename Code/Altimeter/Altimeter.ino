
#include "MS5637.h"
#include <U8x8lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif


// an instance of the U8X8_SH1106_128X64_NONAME_4W_HW_SPI class called u8x8 is created
// The module (in SPI mode) has 7 pins (which on the Arduino PRO Mini with HW SPI use):
// 1 GND - connect to ground pin on micro (Arduino Pro Mini 5V 16Mhz assumed here)
// 2 VCC - connect to VCC pin on micro (assumed 5V)
// 3 CLK - Clock, connect to Pin 13 on micro (assumed by default) - WIRE 3
// MISO - Master In Slave Out, NOT AVAILABLE HERE - but would be pin 12 on micro - WIRE 2 
// 4 MOSI (SDI?)- Master Out Slave In, connect to pin 11 on micro (assumed by default) - WIRE 1
// 5 RES - Reset, connect to Pin 8 (can change)
// 6 DC - Data/Command, connect to pin 9 (can change) 
// 7 CS - Chip Select, connect to pin 10 (can change) - WIRE 4
U8X8_SH1106_128X64_NONAME_4W_HW_SPI u8x8(/*cs=*/10,/*dc=*/9,/*reset=*/8);

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

// Pin connected to red LED Anode (long+), LED Cathode(short-) connected to 220R resistor
// other side of resistor connected to GND
const int ledPin = A1; // LED pin (15) 

float temp, pressure, altBase, alt, altRel;
char test[17] = "0123456789ABCDEF";
char altstr[17];
char altstr2[17];
char tempstr[17];


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
 
  // setup SH1106 SPI display
  u8x8.begin();
  u8x8.setPowerSave(0);
  u8x8.clearDisplay();
  u8x8.setFlipMode(1);  
  u8x8.setFont(u8x8_font_8x13B_1x2_f); 
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
  
  dtostrf(altRel,16,1,altstr);
  u8x8.drawString(0,0,altstr);

  dtostrf(alt,16,1,altstr2);
  u8x8.drawString(0,2,altstr2);

  u8x8.drawString(0,4,test);

  dtostrf(temp,16,1,tempstr);
  u8x8.drawString(0,6,tempstr);

  delay(500);
}
