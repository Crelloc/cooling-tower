/*
 * * Cooling Tower Project
 * * Author: Thomas Turner, thomastdt@gmail.com
 * * Last Modified: 10-31-18
 * updated by CDW 01/08/2019
 * 
 *  This version uses a number of devices and shields. 
 *  Arduino wireless datalogging shield: for xbee communication only
 *  Arduino RTC and datalogging shield: for SD card logging and RTC
 *  Ruggeduino IO shield: using onboard ADC for TSI flow meter.
 *  ADS1115: on proto shield, used for updraft velocity analog signal.
 *  Digital potentiometer on proto shield: used for controlling brushless motor.  10k pot.
 *  INA219 high-side current sensors, x2 (address 0x44 and 0x41): for measuring 4-20ma current loop from RH sensor 
 *  
 *  To be added: Adafruit HTU21D-F humidity sensor, on its own proto board, for temp and RH in electronics enclosure, fixed at address 0x40
*/

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "RTClib.h"
#include <Adafruit_ADS1015.h>
#include <Adafruit_MCP23017.h>
#include <Adafruit_INA219.h>

#define BUF_SIZE 6
#define INDUSTRIAL_SHIELD_ADC_ADDRESS   0x48
#define SCREW_IN_SHIELD_ADC_ADDRESS     0x4A
#define DIGITAL_POTENTIOMETER_ADDRESS   0x2c
#define RH_CURRENT_LOOP_ADDRESS         0x41
#define TEMPC_CURRENT_LOOP_ADDDRESS     0x44
#define ELECTRONICS_RH_ADDRESS          0x40

static Adafruit_MCP23017 mcp1;
static Adafruit_ADS1115 ads(SCREW_IN_SHIELD_ADC_ADDRESS);
static Adafruit_ADS1015 ads_i(INDUSTRIAL_SHIELD_ADC_ADDRESS);     /* Use this for the 12-bit version, 73 if the jumper is shorted) */
static Adafruit_INA219 ina219Temp(0x44);
static Adafruit_INA219 ina219RH(0x41);


typedef struct Log_Pck_Struct {  // Typedef for data packet
    bool isSampling;
    float rh;
    float tempC;
    double updraftVel;
    double inlineFlow;
    float nozzleVel;
    int motorcommand;
    float enclosureTempC;
    float enclosureRH;
} Log_Pck_Struct;
 
/**Global Variables*/
static char g_cmdBuffer[32]                   = {};     //buffer for command 
static char g_cmdIndex                        =  0;     //cmdBuffer index to store command sent from another controller
static double g_updraftVelBuf[BUF_SIZE]       = {};     //updraft velocity ring buffer
static double g_inlineFlowBuf[BUF_SIZE]       = {};     //inline flow ring buffer
static Log_Pck_Struct log_pck                 = {};     //packet of information
RTC_PCF8523     rtc;
static double      g_tempC_inline;                      //inline temperature reading
static double      g_RH_inline;                         //relative humidity
static double      g_RH_enclosure;                      //RH in electronics enclosure
static double      g_tempC_enclosure;                   //Temperature in electronics enclosure
static uint8_t  g_buffer_index                = 0;      //buffer index for ring buffers (g_updraftVelBuf and g_inlineFlowBuf)
static float                   g_updraft_sum  = 0;
static float                    g_inline_sum  = 0;
/**Variables used by interrupts*/
static volatile int   g_Lcycles               = 0;  /**< # of cycles for timer1 */
static volatile int   g_Scycles               = 0;  /**< # of cycles for timer1 */
static volatile bool  g_update_flag           = 0;  /**< flag to calculate velocity, flow, and motor command*/


void setup() 
{                                              
    while (!Serial) {                            /** Open serial communications and wait for port to open: */
        ;                                        /**< wait for serial port to connect. Needed for native USB port only */
    }
    Serial.begin(9600);

    //rugged industrial shield section below
    mcp1.begin(1);                               // use default address 0; used for rugged shield
    mcp1.pinMode(0, OUTPUT);                     //output 0 from rugged shield will connect to enable pin on motor
    
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
      }
    } 
    /**initialize ADC's*/
    ads.setGain(GAIN_ONE);
    ads_i.setGain(GAIN_ONE);
    ads.begin();
    ads_i.begin();
    
    /** initialize timer1 - 16 bit (65536) */
    noInterrupts();                            // disable all interrupts
    TCCR1A  = 0;
    TCCR1B  = 0;
    TCNT1   = 49911;                           // preload timer
    TCCR1B |= ((1 << CS12)| (1 << CS10)) ;     // prescaler 
    TIMSK1 |= (1 << TOIE1);                    // enable timer overflow interrupt
    interrupts();                              // enable all interrupts
}

/** Timer frequency: 1 cycle per second
     Every 1 cycle read analog sensors in buffer and
     set flag to calculate velocity and flow, and to update motor speed
*/
ISR(TIMER1_OVF_vect)        
{
//#define PERIOD_UPDATE_SENSORS 1
                           
#define PERIOD_LOGDATA  10  /** Every 10 cycles set flag to log data*/
#define PERIOD_SENDDATA 10
    TCNT1 = 49911;
    ++g_Lcycles;               /** number of seconds elapsed; count for logging data*/
    ++g_Scycles;               /** number of seconds elapsed; count for sending data*/
    g_update_flag = 1;
   
}

byte i2c_readRegisterByte (uint8_t deviceAddress, uint8_t registerAddress)
{
    byte registerData;
    Wire.beginTransmission(deviceAddress);    // set sensor target
    Wire.write(registerAddress);              // set memory pointer
    Wire.endTransmission();
       
    Wire.requestFrom( deviceAddress, (uint8_t)1);     // request one byte
    registerData = Wire.read(); 
                                              // you could add more data reads here if you request more than one byte
    return registerData;                      // the returned byte from this function is the content from registerAddress
}

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
/** For zero padding: 
* 
* Format: XXX.XXXX0
* ex: 123.1234 --> 123.12340
*     23.13    --> 023.13000
*/
static void sprintf_f(float fval, char *c)
{
    char *tmpSign = (fval < 0) ? "-"   : "";
    float tmpVal  = (fval < 0) ? -fval : fval;

    int tmpInt1   = tmpVal;
    float tmpFrac = tmpVal - tmpInt1;
    int tmpInt2 = trunc(tmpFrac * 10000);

    sprintf(c, "%s%03d.%04d0", tmpSign, tmpInt1, tmpInt2);
   
}
/**will log to sd card*/
static int log_info(Log_Pck_Struct *pck) 
{
    int ret = 0;
    char buf[10];
    static bool initialized = 0;
    File dataFile           = SD.open("datalog.txt", FILE_WRITE);
    DateTime now            = rtc.now();
    int year                = now.year();
    int month               = now.month();
    int day                 = now.day();
    int hour                = now.hour();
    int minute              = now.minute();
    int second              = now.second();
    
    // if the file is available, write to it:
    if (dataFile) {
        if (!initialized) {
           initialized = true;
           dataFile.println("time, sampling, rh(%), tempC, v_updraft, inlineFlow, v_nozzle, motorcmd");
        }
        dataFile.print(year, DEC);
        dataFile.print('/');
        dataFile.print(month, DEC);
        dataFile.print('/');
        dataFile.print(day, DEC);
        dataFile.print(' ');
        dataFile.print(hour, DEC);
        dataFile.print(':');
        dataFile.print(minute, DEC);
        dataFile.print(':');
        dataFile.print(second, DEC);
        dataFile.print(", ");
        dataFile.print(pck->isSampling);
        dataFile.print(", ");
        dataFile.print(pck->rh, 4);
        dataFile.print(", ");
        dataFile.print(pck->tempC, 2);
        dataFile.print(", ");
        dataFile.print(pck->updraftVel, 4);
        dataFile.print(", ");
        dataFile.print(pck->inlineFlow, 4);
        dataFile.print(", ");
        dataFile.print(pck->nozzleVel, 4);
        dataFile.print(", ");
        dataFile.println(pck->motorcommand);
        dataFile.close();

    } else {
        Serial.println("error opening datalog.txt");
        ret = -1;
    }


    /**print to the serial port too:
     * if wireless sd shield used, then the following 
     * will send through the xbee.
     */
    Serial.print("Time = ");
    Serial.print(year, DEC);
    Serial.print('/');
    sprintf(buf, "%02d", month);
    Serial.print(buf);
    Serial.print('/');
    sprintf(buf, "%02d", day);
    Serial.print(buf);
    Serial.print(' ');
    sprintf(buf, "%02d", hour);
    Serial.print(buf);
    Serial.print(':');
    sprintf(buf, "%02d", minute);
    Serial.print(buf);
    Serial.print(':');
    sprintf(buf, "%02d", second);
    Serial.print(buf);
    Serial.print(", Sampling = ");
    Serial.print(pck->isSampling);
    Serial.print(", rh = (%)");
    sprintf_f(pck->rh, buf);
    Serial.print(buf);
    Serial.print(", tempC = ");
    sprintf_f(pck->tempC, buf);
    Serial.print(buf);
    Serial.print(", v_updraft = ");
    sprintf_f(pck->updraftVel, buf);
    Serial.print(buf);
    Serial.print(", inlineFlow = ");
    sprintf_f(pck->inlineFlow, buf);
    Serial.print(buf);
    Serial.print(", v_nozzle = ");
    sprintf_f(pck->nozzleVel, buf);
    Serial.print(buf);
    Serial.print(", motorcmd = ");
    sprintf(buf, "%03d", pck->motorcommand);
    Serial.print(buf);
    Serial.println();
    return ret;
}

/**send via wireless signal
 * currently not used because we are 
 * using sd wireless shield which will
 * send packet through arduino's default
 * serial ports.
 */
static int send_pkt(Log_Pck_Struct *pck) 
{
    return 0;
}
/* Function get_stringcmd
 * Purpose: retrieve and store commands that have been sent from another controller.
 */
static uint8_t get_stringcmd()
{
    char c;
    bool flag = 0;
    while(Serial.available()){ //while there's a chacter in the serial buffer
        c = Serial.read();
        if(g_cmdIndex == 64){/*error check for buffer overflow*/}  
        if(c == '\r' || c == '\n'){ //when termination character has been read, 
            g_cmdBuffer[g_cmdIndex] = '\0';  //character arrays should have a terminating null character at the end of string 
            g_cmdIndex = 0; //rest index to store character
            flag = 1; //enable flag for command to be executed
            break;
        } else {
            g_cmdBuffer[g_cmdIndex] = c;  //store character in array
        }
        g_cmdIndex++;  //move to next position
         
    }
    return flag;
}
/* Function: execute_cmd
 * Info: execute list of commands:
 * Commands:    
 *          SA: set isSampling variable: bool value (0 or 1)
 *          U:  update/write to digital potentiometer to control motor speed
 */
static int execute_cmd(void* val, char const* cmd)
{
    if(strcmp(cmd, "SA")==0){
        log_pck.isSampling = *(int*)val;  
        mcp1.digitalWrite(0, *(int*)val);                   //output val to digital output 0 of industrial shield                        
        
    } else if(strcmp(cmd, "U")==0){
        i2c_writeRegisterByte (DIGITAL_POTENTIOMETER_ADDRESS, 16, *(uint8_t*)val);  //device address, instruction byte, pot value 
    }
    return 0;  
}

void parse_stringcmd(char* buf)
{
    char cmd[2] = {}; //cmd is at most 2 characters, ie, SA
    int i = -1; //-1 is a placeholder
    sscanf(buf, "%s %d", cmd, &i); //the data in buf would be copied to variables cmd and i 
    execute_cmd(&i, (char const*)cmd);
}

/**
 * Compute moving average and add sample to ring buffer
*/
double movingAverage(double *Arr, float *Sum, volatile int pos, int len, double num)
{
    *Sum     = *Sum - Arr[pos] + num;
    Arr[pos] = num;
    return *Sum / len;
}

void update_sensors()        
{
    double error;
    float  iso_nozzle_diameter = .0031f;   // isokinetic nozzle diameter in meters
    float tempC                = map(g_tempC, 4.0f, 20.0f, 0.0f, 100.0f);
    /**<
     * updraft velocity read:
     * adc value * multiplier for gain 1 of ads1115 * mph coefficient
     * ADS1115 @ +/- 4.096V gain (16-bit results)
     * Output is in mph 
     */
    double updraft_v = ads.readADC_Differential_0_1() * 0.125f * 0.018; //[adc value * Gain 1 coeff * m/s coeff], velocity in m/s
    /**< 
     * inline flow read: 
     * adc value * multiplier for gain 1 of ads1015 * 2 [because voltage into adc has doubled from 5v to 10v]
     * Output is in mV but converted to L/min 
     */
    double inline_f  = map(ads_i.readADC_SingleEnded(0) * 2.0f * 2, 0.0, 10000.0, 0.0, 200.0) 
                                  * (273.15 + tempC) / (273.15 + 21.11); //units = liters/min

    /**store velocity and flow in ring buffer.*/
    if(g_buffer_index == BUF_SIZE){
        g_buffer_index = 0;  
    }
    
    log_pck.updraftVel      = movingAverage(g_updraftVelBuf, &g_updraft_sum, g_buffer_index,BUF_SIZE, updraft_v);
    log_pck.inlineFlow      = movingAverage(g_inlineFlowBuf, &g_inline_sum, g_buffer_index,BUF_SIZE, inline_f );
    
    /*calulate nozzle velocity*/
    log_pck.nozzleVel       = log_pck.inlineFlow /60/1000/ (5*(3.1415*pow((iso_nozzle_diameter/2),2))); //units = m/s
    
    log_pck.tempC           = tempC;
    log_pck.rh              = tempC;

    /*P control for motor*/
     error        = log_pck.nozzleVel - log_pck.updraftVel; //units = m/s
#define KP 2
#define KI 1
#define KD 1
    log_pck.motorcommand = log_pck.motorcommand + KP*error;  //recall that 255 = motor off, 0 = full speed. positive error means motor is spinning too fast.
    //log_pck.motorcommand = (KP * error) + (KI * integral) + (KD * derivative); //integral and derivative is not defined
    if(log_pck.motorcommand > 255) log_pck.motorcommand = 255;
    else if(log_pck.motorcommand < 0) log_pck.motorcommand = 0;
    
    if(log_pck.isSampling){//if we want to sample...
        execute_cmd(&log_pck.motorcommand, "U");
    }
    else{//make sure motors are off
        log_pck.motorcommand    = 0;
        execute_cmd(&log_pck.motorcommand, "SA");
    }
    g_buffer_index++;
}

void tests()
{
//    while(1);
}
void loop()
{
    //poll until we want to update sensors or store command from another controller
    while(!g_update_flag && Serial.available() == 0 );

    if(Serial.available() > 0){ //if serial buffer received command
        if(get_stringcmd()){//if we received full command
//            Serial.print("cmd: ");
//            Serial.println(g_cmdBuffer);
            parse_stringcmd(g_cmdBuffer);  //parse and execute command
        }
    }
    if(g_update_flag){//if it's time to read sensors
      
        g_update_flag = 0;
        update_sensors(); 

        if(g_Scycles >= PERIOD_SENDDATA){
            g_Scycles = 0;
            send_pkt(&log_pck);      
        }
        if(g_Lcycles >= PERIOD_LOGDATA){   
            g_Lcycles = 0;
            log_info(&log_pck);
        }
    }
//      tests(); //function used to test other functions.
}
