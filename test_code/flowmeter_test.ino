#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "RTClib.h"
#include <Adafruit_ADS1015.h>
#include <Adafruit_MCP23017.h>

static Adafruit_MCP23017 mcp1;
static Adafruit_ADS1115 ads(0x4A);
static Adafruit_ADS1015 ads_i(0x48);     /* Use this for the 12-bit version, 73 if the jumper is shorted) */


byte i2c_writeRegisterByte (uint8_t deviceAddress, uint8_t registerAddress, uint8_t newRegisterByte)
{
    byte result;
    Wire.beginTransmission(deviceAddress); 
    Wire.write(registerAddress);  
    Wire.write(newRegisterByte); 
    result = Wire.endTransmission();    // Wire.endTransmission(); returns 0 if write operation was successful
                                        // delete this comment – it was only needed for blog layout.
    //delay(5);  // optional:  some sensors need time to write the new data, but most do not. Check Datasheet.
    if(result > 0){
      Serial.print("FAIL in I2C register write! Error code:");
      Serial.println(result);
    }    
    
    return result;    // the returned value from this function could be tested as shown above
    //it’s a good idea to check the return from Wire.endTransmission() the first time you write to a sensor 
    //if the first test is okay (result is 0), then I2C sensor coms are working and you don’t have to do extra tests
} 

void update_sensors()        
{
    int error;
    int motorcommand;
    double nozzleVel; 
    uint16_t gainfactor        = 1;        // 1 is placeholder
    float  iso_nozzle_diameter = 3.1f;   // isokinetic nozzle diameter in millimeters
    float tempC                = 20.0f; //placeholder
    double updraft_v           = 10.0; //velocity in mph
    /**< 
     * inline flow read: 
     * adc value * multiplier for gain 1 of ads1015 * 2 [because voltage into adc has doubled from 5v to 10v]
     * Output is in mV but converted to L/min 
     */
    double inline_f  = map(ads_i.readADC_SingleEnded(0) * 2.0f * 2, 0.0, 10000.0, 0.0, 200.0) 
                                  * (273.15 + tempC) / (273.15 + 21.11);

    nozzleVel        = inline_f / 5*(3.1415*pow((iso_nozzle_diameter/2),2));

    error = gainfactor*(nozzleVel-updraft_v);
    if(error < 0){
        //slow down motors
        motorcommand -= 10;
        if(motorcommand < 0)
          motorcommand = 0;
    }else if(error+motorcommand > 255){
        motorcommand = 255;
    } else
        motorcommand+=error; 
    //output command to motor
    i2c_writeRegisterByte (0x2c, 16, motorcommand);  //device address, instruction byte, pot value 
 

}

void setup() {

    while (!Serial) {                            /** Open serial communications and wait for port to open: */
        ;                                        /**< wait for serial port to connect. Needed for native USB port only */
    }
    Serial.begin(9600);

    //rugged industrial shield section below
    mcp1.begin(1);                               //used for rugged shield
    mcp1.pinMode(0, OUTPUT);                     //NOT USED YET; output 0 from rugged shield will connect to enable pin on motor; 
    
    ads.setGain(GAIN_ONE);
    ads_i.setGain(GAIN_ONE);
    ads.begin();
    ads_i.begin();

    //make sure motor is off
    i2c_writeRegisterByte (0x2c, 16, 0);  //device address, instruction byte, pot value 

}

void loop() {

    update_sensors();
    delay(2000);
}
