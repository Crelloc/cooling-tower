#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "RTClib.h"
#include <Adafruit_ADS1015.h>
#include <Adafruit_MCP23017.h>

static Adafruit_MCP23017  mcp1;
static Adafruit_ADS1115   ads(0x4A);
static Adafruit_ADS1015   ads_i(0x48);     /* Use this for the 12-bit version, 73 if the jumper is shorted) */
RTC_PCF8523               rtc;
static double             last_error;
static double             integral = 0;
static double             derivative;

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

void logdata(double vel)
{
    File dataFile = SD.open("PID.txt", FILE_WRITE);
    DateTime now = rtc.now();

    if (dataFile) {//log to SD
        dataFile.print(now.year(), DEC);
        dataFile.print('/');
        dataFile.print(now.month(), DEC);
        dataFile.print('/');
        dataFile.print(now.day(), DEC);
        dataFile.print(' ');
        dataFile.print(now.hour(), DEC);
        dataFile.print(':');
        dataFile.print(now.minute(), DEC);
        dataFile.print(':');
        dataFile.print(now.second(), DEC);
        dataFile.print(", ");
        dataFile.println(vel);
        dataFile.close();

    } else {
        Serial.println("error opening PID.txt");
    }
    //print to console
    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(' ');
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.print(", ");
    Serial.println(vel);
}

void update_sensors()        
{
    double        error;
    double        motorcommand;
    double        nozzleVel; 
    const float   iso_nozzle_diameter = 3.1f;   // isokinetic nozzle diameter in millimeters
    const float   tempC                = 20.0f; //placeholder
    const double  updraft_v           = 10.0; //velocity in mph
    /**< 
     * inline flow read: 
     * adc value * multiplier for gain 1 of ads1015 * 2 [because voltage into adc has doubled from 5v to 10v]
     * Output is in mV but converted to L/min 
     */
    double inline_f = map(ads_i.readADC_SingleEnded(0) * 2.0f * 2, 0.0, 10000.0, 0.0, 200.0) 
                                  * (273.15 + tempC) / (273.15 + 21.11);

    nozzleVel    = inline_f / 5*(3.1415*pow((iso_nozzle_diameter/2),2));
    error        = nozzleVel - updraft_v;
    integral     = integral + error;
    derivative   = error - last_error;
#define KP 1
#define KI 1
#define KD 1
    motorcommand = (KP * error) + (KI * integral) + (KD * derivative);
    if(motorcommand > 255) motorcommand = 255;
    else if(motorcommand < 0) motorcommand = 0;
 
    //output command to motor
    i2c_writeRegisterByte (0x2c, 16, (uint8_t)motorcommand);  //device address, instruction byte, pot value 
    last_error = error;

    //log nozzleVel and time
    logdata(nozzleVel);
}

void setup() {

    while (!Serial) {                            /** Open serial communications and wait for port to open: */
        ;                                        /**< wait for serial port to connect. Needed for native USB port only */
    }
    Serial.begin(9600);

    //RTC section below
    if (! rtc.begin()) {
        Serial.println("Couldn't find RTC");
        while (1);
    }
    if (! rtc.initialized()) {
        Serial.println("RTC is NOT running!");
        // following line sets the RTC to the date & time this sketch was compiled
        // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        // This line sets the RTC with an explicit date & time, for example to set
        // January 21, 2014 at 3am you would call:
        // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
    }
    
    {//SD Card section below
      Serial.print("\nInitializing SD card...");
                                                 // make sure that the default chip select pin is set to
                                                 // output, even if you don't use it:
      pinMode(10, OUTPUT);                          /**Wireless SD shield uses pin 4 as chipselect, 
                                                        Datalogging shield uses pin 10 as chipselect*/
                                                   // we'll use the initialization code from the utility libraries
                                                   // since we're just testing if the card is working!
      if (!SD.begin(10)) {
          Serial.println("initialization failed. Things to check:");
          Serial.println("* is a card inserted?");
          Serial.println("* is your wiring correct?");
          Serial.println("* did you change the chipSelect pin to match your shield or module?");
  
      } else {
          Serial.println("Wiring is correct and a card is present.");
          {//print header
            File dataFile = SD.open("PID.txt", FILE_WRITE);
            if(dataFile){
                dataFile.println("time, nozzle_vel");
                dataFile.close();
                
            } else{
                Serial.println("Error creating or opening datalog.txt!");
            }
            Serial.println("time, nozzle_vel");
          }
      }
    } 
    
    //rugged industrial shield section below
    mcp1.begin(1);                               //used for rugged shield
    mcp1.pinMode(0, OUTPUT);                     //NOT USED YET; output 0 from rugged shield will connect to enable pin on motor; 
    
    ads.setGain(GAIN_ONE);
    ads_i.setGain(GAIN_ONE);
    ads.begin();
    ads_i.begin();

    //make sure motor is not running
    i2c_writeRegisterByte (0x2c, 16, 0);  //device address, instruction byte, pot value 

}

void loop() {

    update_sensors();
    delay(2000);
}
