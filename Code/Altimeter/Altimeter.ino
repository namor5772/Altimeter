
#include "MS5637.h"
#include <U8x8lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif


// an instance of the U8X8_SH1106_128X64_NONAME_4W_HW_SPI class called u8x8 is created
U8X8_SH1106_128X64_NONAME_4W_HW_SPI u8x8(/*cs=*/10,/*dc=*/9,/*reset=*/8);
// an instance of the BaroSensorClass called BaroSensor is created
BaroSensorClass BaroSensor;

// digital pin 7 has a pushbutton attached to it. Give it a name:
int pushButton = 7;
float temp, pressure, altBase, alt, altRel;
char altstr[9];
char altstr2[9];
char tempstr[9];


void setup()
{
  Serial.begin(9600);

  // make the pushbutton's pin an input:
  pinMode(pushButton, INPUT);
 
  // setup MS5637 sensor (An instance of the BaroSensorClass object BaroSensor has been constructed in MS5637.cpp)
  BaroSensor.begin();
  BaroSensor.dumpDebugOutput();
  BaroSensor.getTempAndPressure(&temp, &pressure);
  altBase = BaroSensor.pressure2altitude(pressure);
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
  // read the input pin to toggle LED:
  int buttonState = digitalRead(pushButton);

/*
  if (!BaroSensor.isOK()) {
    // Try to reinitialise the sensor if we can
    BaroSensor.begin();
    BaroSensor.getTempAndPressure(&temp, &pressure);
  }
  else */ if (buttonState==1) {
    // Zero the altimeter (when button is pressed)
    BaroSensor.getTempAndPressure(&temp, &pressure);
    altBase = BaroSensor.pressure2altitude(pressure);
  }
  else {
    BaroSensor.getTempAndPressure(&temp, &pressure);
  }

  // Calculates the altitude after zeroing
  alt = BaroSensor.pressure2altitude(pressure);
  altRel = alt - altBase;
  //Serial.print(BaroSensor.getTemperature());
  //Serial.print(" ");
  Serial.print(alt);
  Serial.print(" ");
  Serial.println(altRel);
  
  // Display the data
  
  dtostrf(altRel,8,1,altstr);
  u8x8.drawString(0,0,altstr);

  dtostrf(alt,8,1,altstr2);
  u8x8.drawString(0,2,altstr2);

  dtostrf(temp,8,1,tempstr);
  u8x8.drawString(0,6,tempstr);

  delay(500);
}
