  
/*
https://www.rugged-circuits.com/24v-industrial-tech-page
This sketch, intended to provide insight on basic functionality of the shield, 
sets up the ADC and I/O expander, with Input 0 (B0) set up as an input, and A0
set up as an output.  The output will mirror the input (with a 1 second delay),
and will also spit out values from the ADC (all 4 channels).  The values that
are non-zero are due to the tiny amount of leakage through the clamping diode,
but any input will overcome the leakage. 
*/

#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <Adafruit_MCP23017.h>

Adafruit_MCP23017 mcp1;
Adafruit_ADS1015 ads1(72);     /* Use this for the 12-bit version, 73 if the jumper is shorted) */


void setup() {  
  mcp1.begin(1);      // use default address 0

  Serial.begin(9600);
  
  ads1.setGain(GAIN_ONE);
  ads1.begin();
  mcp1.pinMode(0, OUTPUT);
  mcp1.pinMode(8, INPUT);

}



void loop() {
  int16_t adc0, adc1, adc2, adc3;

  if(mcp1.digitalRead(8) == 0)
  {
    mcp1.digitalWrite(0, HIGH);
  }
  else 
  {
    mcp1.digitalWrite(0, LOW);
  }
  
  adc0 = ads1.readADC_SingleEnded(0);
  adc1 = ads1.readADC_SingleEnded(1);
  adc2 = ads1.readADC_SingleEnded(2);
  adc3 = ads1.readADC_SingleEnded(3);
  Serial.print("AIN0: "); Serial.println(adc0);
  Serial.print("AIN1: "); Serial.println(adc1);
  Serial.print("AIN2: "); Serial.println(adc2);
  Serial.print("AIN3: "); Serial.println(adc3);
  Serial.println(" ");
  
  delay(1000);

}
