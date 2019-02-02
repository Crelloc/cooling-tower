/* Last update 12/19/18
 *  This version uses a number of devices and shields. 
 *  Arduino wireless datalogging shield: for xbee communication only
 *  Arduino RTC and datalogging shield: for SD card logging and RTC
 *  Ruggeduino IO shield: using onboard ADC for TSI flow meter.
 *  ADS1115: on proto shield, used for updraft velocity analog signal.
 *  Digital potentiometer on proto shield: used for controlling brushless motor.  10k pot.
 *  INA219 high-side current sensors, x2 (address 0x40 and 0x41): for measuring 4-20ma current loop from RH sensor 
 *  
 *  To be added: Adafruit HTU21D-F humidity sensor, on its own proto board, for temp and RH in electronics enclosure.
 * 
 * 
 * 
 */

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "RTClib.h"
#include <Adafruit_ADS1015.h>
#include <Adafruit_MCP23017.h>
#include <Adafruit_MCP4725.h>
#include <Adafruit_INA219.h> //current sensors

#define BUF_SIZE 6
#define ADC2_ADDRESS                    0x48
//#define INDUSTRIAL_SHIELD_GPIO_ADDRESS  0x21
#define SCREW_IN_SHIELD_ADC_ADDRESS     0x4A
#define DIGITAL_POTENTIOMETER_ADDRESS   0x2c
#define RH_CURRENT_LOOP_ADDRESS         0x41
#define TEMPC_CURRENT_LOOP_ADDDRESS     0x44
#define ELECTRONICS_RH_ADDRESS          0x40 
#define DAC_ADDRESS                     0x62

#define SD_CARD_CS_PIN                  9 //not factory, must modify jumper on board to make it 9
#define MOTOR_ENABLE_PIN                6 //digital pin to control relay shield for motor enable.  Pin 6 is relay 2 on relay shield.


static Adafruit_ADS1115 ads(SCREW_IN_SHIELD_ADC_ADDRESS);
static Adafruit_ADS1115 ads_i(ADC2_ADDRESS);     /* new shield with ADS 1115 to measure TSI flowmeter */
static Adafruit_INA219 ina219Temp(0x44);  
static Adafruit_INA219 ina219RH(0x41);
static Adafruit_MCP4725 dac;
RTC_PCF8523               rtc;
static double             last_error;
static double             integral = 0;
static double             derivative;
static double             motorcommand = 0;
static double             last_motorcommand = 0;


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
    double        nozzleVel; 
    const float   iso_nozzle_diameter = .0031f;   // isokinetic nozzle diameter in meters
    float   tempC                     = (ina219Temp.getCurrent_mA()-4)/16*100; //get 4-20ma signal and convert to 0-100C scale.
    double  updraft_v                 = 12.0; //12 m/s is the max speed for p control to work
#define MAP(x, a, b, c, d) \
    ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
       __typeof__ (c) _c = (c); \
       __typeof__ (d) _d = (d); \
       __typeof__ (x) _x = (x); \
    ((_x - _a) / (_b - _a) * (_d - _c) + _c); })
    /**< 
     * inline flow read: 
     * adc value * multiplier for gain 1 of ads1015 * 2 [because voltage into adc has doubled from 5v to 10v]
     * Output is in mV but converted to L/min 
     */
    double inline_f = MAP(ads_i.readADC_SingleEnded(1) * 0.1875f * 2, 0.0, 10000.0, 0.0, 200.0) 
                                  * (273.15 + tempC) / (273.15 + 21.11); //units = liters/min
    Serial.print(" inline flow = ");
    Serial.println(inline_f);

    nozzleVel    = inline_f/60/1000/ (5*(3.1415*pow((iso_nozzle_diameter/2),2))); //units = m/s
    error        = nozzleVel-updraft_v ; //units = m/s
    Serial.print("error = ");
    Serial.println(error);
    integral     = integral + error;
    derivative   = error - last_error;
#define KP -100
#define KI 1
#define KD 1
#define DELTA 50
    motorcommand = motorcommand + KP*error;  
    //motorcommand = (KP * error) + (KI * integral) + (KD * derivative);
    if(last_motorcommand == 4095) motorcommand -= DELTA;
    else if(motorcommand > 4095) motorcommand = 4095;
    else if(last_motorcommand == 0) mototcommand = += DELTA;
    else if(motorcommand < 0) motorcommand = 0;

    //output command to motor
    dac.setVoltage(motorcommand, false);
    last_error = error;
    last_motorcommand = motorcommand;
    //log nozzleVel and time
    //logdata(nozzleVel);
    Serial.print("motor command = ");
    Serial.println(motorcommand);
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
      pinMode(9, OUTPUT);                          /**Wireless SD shield uses pin 4 as chipselect, 
                                                        Datalogging shield uses pin 10 as chipselect*/
                                                   // we'll use the initialization code from the utility libraries
                                                   // since we're just testing if the card is working!
      if (!SD.begin(9)) {
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
    
   // initialize digital pin for motor enable as an output.
    pinMode(MOTOR_ENABLE_PIN, OUTPUT);
    
    ads.setGain(GAIN_ONE);
    ads_i.setGain(GAIN_TWOTHIRDS);
    ads.begin();
    ads_i.begin();
    
    ina219Temp.begin();
    ina219Temp.setCalibration_32V_20mA(); //may have to edit library to invoke 32v_20mA.  otherwise, initialize as 32v_1A and divide results by 50.
  // For Adafruit MCP4725A1 the address is 0x62 (default) or 0x63 (ADDR pin tied to VCC)
  // For MCP4725A0 the address is 0x60 or 0x61
    // For MCP4725A2 the address is 0x64 or 0x65
    dac.begin(0x62);

    //make sure motor is not running
    dac.setVoltage(0, false);
}

void loop() {

    update_sensors();
    delay(2000);
}
