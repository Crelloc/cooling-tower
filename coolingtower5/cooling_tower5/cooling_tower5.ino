/*
 * * Cooling Tower Project
 * * Author: Thomas Turner, thomastdt@gmail.com
 * * Last Modified: 02-11-19
 * 
 * 
 * 
 *  This version uses a number of devices and shields. 
 *  
 *  Arduino RTC and datalogging shield: for SD card logging (SPI) and RTC (0x68).  SD Card chip select has been changed in HW to pin 9.
 *  Ruggeduino IO shield: using onboard ADC for TSI flow meter. Using channel 0 in single-ended, gain one, for TSI flowmeter. Use voltage divider jumper on shield to reduce 0-10v to 0-5v. use single ended mode, since differential mode apparently goes negative sometimes.
 *  ADS1115: on proto shield, used for updraft velocity analog signal. set to gain one, using differential read between channel 0-1. Address 0x4a selected
 *  Digital potentiometer on proto shield: used for controlling brushless motor.  10k pot.
 *  INA219 high-side current sensors, x2 (address 0x44 and 0x41): for measuring 4-20ma current loop from RH sensor 
 *  HTU21D-F temperature and humidity sensor for inside electrical enclosure, address 0x40
 *  This version relies on a sparkfun zigbee shield.  For the arduino mega, the Din and Dout must be connecte to pins 10 and 11, insted of default 2 and 3.
 *  
 *  To be added: Adafruit HTU21D-F humidity sensor, on its own proto board, for temp and RH in electronics enclosure, fixed at address 0x40
*/

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "RTClib.h"
#include <Adafruit_ADS1015.h> //ruggeduino industrial shield adc
#include <Adafruit_MCP23017.h> //ruggeduino industrial shield gpio
#include <Adafruit_INA219.h> //current sensors
#include <SoftwareSerial.h> //for xbee shield
#include "Adafruit_HTU21DF.h" //enclosure temp and humidity sensor
#include <Adafruit_MCP4725.h> //for dac that's connected to motor controller

#define CMD_BUF_SIZE                    32
#define BUF_SIZE                        6
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

#define MAP(x, a, b, c, d) \
    ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
       __typeof__ (c) _c = (c); \
       __typeof__ (d) _d = (d); \
       __typeof__ (x) _x = (x); \
    ((_x - _a) / (_b - _a) * (_d - _c) + _c); })

#define CHECK_MOTOR_CMD(CMD) \
    do { \
        if(*CMD > 4095) *CMD = 4095; \
        else if(*CMD < 0) *CMD = 0; \
    } while(0);
//Pin usage
//D10, D11, D12, D13 are SPI pins. 10 is used for Chip Select by the SD card.  However, these are all jumperable.  as of 1/24 modified to be CS 9.
//A4 and A5 are I2c pins
//possible software serial pins: (Mega): 10, 11, 12, 13, 14, 15, 50, 51, 52, 53, A8 (62), A9 (63), A10 (64), A11 (65), A12 (66), A13 (67), A14 (68), A15 (69)



static Adafruit_MCP23017 mcp1;
static Adafruit_ADS1115 ads(SCREW_IN_SHIELD_ADC_ADDRESS);
static Adafruit_ADS1115 ads_i(ADC2_ADDRESS);     /* new shield with ADS 1115 to measure TSI flowmeter */
static Adafruit_INA219 ina219Temp(0x44);  
static Adafruit_INA219 ina219RH(0x41);
static Adafruit_HTU21DF htu = Adafruit_HTU21DF();
static Adafruit_MCP4725 dac;


//For Atmega2560, ATmega32U4, etc.
// XBee's DOUT (TX) is connected to pin 10 (Arduino's Software RX)
// XBee's DIN (RX) is connected to pin 11 (Arduino's Software TX)
SoftwareSerial XBee(10, 11); // RX, TX


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
    float encl_tempC;
    float encl_rh;
    int8_t mode;
} Log_Pck_Struct;
 
/**Global Variables*/
static char g_cmdBuffer[CMD_BUF_SIZE]         = {};     //buffer for command 
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
static int g_motor_control_mode               =0;  // 0 is PID feedback control (default), 1 is user defined using serial commands

/**Variables used by interrupts*/
static volatile int   g_Lcycles               = 0;  /**< # of cycles for timer1 */
static volatile int   g_Scycles               = 0;  /**< # of cycles for timer1 */
static bool  g_update_flag           = 0;  /**< flag to calculate velocity, flow, and motor command*/



void setup() 
{                                              
    while (!Serial) {                            /** Open serial communications and wait for port to open: */
        ;                                        /**< wait for serial port to connect. Needed for native USB port only */
    }
    Serial.begin(9600);
    XBee.begin(9600);
    Serial.flush();
    XBee.flush();

    //rugged industrial shield section below
    
    //RTC section below
    if (! rtc.begin()) {
        Serial.println("Couldn't find RTC");
        //while (1);
    }

    if (!htu.begin()) {
    Serial.println("Couldn't find enclosure temp and RH sensor!");
    }

    dac.begin(0x62);

    if (! rtc.initialized()) {
        Serial.println("RTC is NOT running!");
        // following line sets the RTC to the date & time this sketch was compiled
        // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        // This line sets the RTC with an explicit date & time, for example to set
        // January 21, 2014 at 3am you would call:
        // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
    }
    //rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); //uncomment this line to sync rtc with computer time
    {//SD Card section below
      Serial.print("\nInitializing SD card...");
                                                 // make sure that the default chip select pin is set to
                                                 // output, even if you don't use it:
      pinMode(SD_CARD_CS_PIN , OUTPUT);                          /**Wireless SD shield uses pin 4 as chipselect, 
                                                        Datalogging shield uses pin 10 as chipselect*/
                                                   // we'll use the initialization code from the utility libraries
                                                   // since we're just testing if the card is working!
      if (!SD.begin(SD_CARD_CS_PIN )) {
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
    ads_i.setGain(GAIN_TWOTHIRDS); // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default).  TSI flowmeter has an output of 10V.  We divide in half with voltage divider, but need a range of 0-5V.
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

    /**initialize current sensors**/
    ina219Temp.begin();
    ina219RH.begin();
    ina219Temp.setCalibration_32V_20mA(); //may have to edit library to invoke 32v_20mA.  otherwise, initialize as 32v_1A and divide results by 50.
    ina219RH.setCalibration_32V_20mA();

    
   // initialize digital pin for motor enable as an output.
    pinMode(MOTOR_ENABLE_PIN, OUTPUT);
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

//byte i2c_readRegisterByte (uint8_t deviceAddress, uint8_t registerAddress)
//{
//    byte registerData;
//    Wire.beginTransmission(deviceAddress);    // set sensor target
//    Wire.write(registerAddress);              // set memory pointer
//    Wire.endTransmission();
//       
//    Wire.requestFrom( deviceAddress, (uint8_t)1);     // request one byte
//    registerData = Wire.read(); 
//                                              // you could add more data reads here if you request more than one byte
//    return registerData;                      // the returned byte from this function is the content from registerAddress
//}
//
//byte i2c_writeRegisterByte (uint8_t deviceAddress, uint8_t registerAddress, uint8_t newRegisterByte)
//{
//    byte result;
//    Wire.beginTransmission(deviceAddress); 
//    Wire.write(registerAddress);  
//    Wire.write(newRegisterByte); 
//    result = Wire.endTransmission();    // Wire.endTransmission(); returns 0 if write operation was successful
//    //delay(5);  // optional:  some sensors need time to write the new data, but most do not. Check Datasheet.
//    if(result > 0){
//      Serial.print("FAIL in I2C register write! Error code:");
//      Serial.println(result);
//    }    
//    
//    return result;    // the returned value from this function could be tested as shown above
//    //it’s a good idea to check the return from Wire.endTransmission() the first time you write to a sensor 
//    //if the first test is okay (result is 0), then I2C sensor coms are working and you don’t have to do extra tests
//} 
/** For zero padding: 
* 
* Format: XXX.XXXX0
* ex: 123.1234 --> 123.12340
*     23.13    --> 023.13000
*/
static void sprintf_f(float fval, char *c)
{
    char *tmpSign = (fval < 0) ? (char*)"-"   : (char*)"";
    float tmpVal  = (fval < 0) ? -fval : fval;

    int tmpInt1   = tmpVal;
    float tmpFrac = tmpVal - tmpInt1;
    int tmpInt2 = trunc(tmpFrac * 10000);

    sprintf(c, "%s%03d.%04d0", tmpSign, tmpInt1, tmpInt2);
   
}
static void SendToSD(char * buf){
    static bool initialized = 0;
    File dataFile = SD.open("datalog.txt", FILE_WRITE);
    
    if (dataFile) {
        if (!initialized) {
            initialized = true;
            dataFile.println("time, sampling, rh(%), tempC, v_updraft, inlineFlow, "
                             "v_nozzle, motorcmd, enclosure RH, enclosure TempC");
        }
        dataFile.println(buf);
        dataFile.close();
    }else{ Serial.println("Error opening datalog.txt!");}
}

static void send_log(Log_Pck_Struct *pck, uint8_t sel){// sel = 0 for XBee; sel = 1 for SD
    char buf[500];
    char rh[10];
    char tempC[10];
    char updraft[10];
    char inlineflow[10];
    char nozzleVel[10];
    char encl_hum[10];
    char encl_temp[10];
    DateTime now            = rtc.now();
    int year                = now.year();
    int month               = now.month();
    int day                 = now.day();
    int hour                = now.hour();
    int minute              = now.minute();
    int second              = now.second();

    sprintf_f(pck->rh, rh);
    sprintf_f(pck->tempC, tempC);
    sprintf_f(pck->updraftVel, updraft);
    sprintf_f(pck->inlineFlow, inlineflow);
    sprintf_f(pck->nozzleVel, nozzleVel);
    sprintf_f(pck->encl_rh, encl_hum);
    sprintf_f(pck->encl_tempC, encl_temp);
    sprintf(buf,"Time = %04d/%02d/%02d %02d:%02d:%02d, "
                "Sampling = %d, "
                "Mode = %d, "
                "rh = (%%)%s, "
                "tempC = %s, "
                "v_updraft = %s, "
                "inlineFlow = %s, "
                "v_nozzle = %s, "
                "motorcmd = %04d, "
                "encl_hum = (%%)%s, "
                "encl_temp = %s ",
                year, month, day, hour, minute, second, pck->isSampling, pck->mode,
                rh, tempC, updraft, inlineflow, nozzleVel, pck->motorcommand,
                encl_hum, encl_temp
            );
    if(sel == 0)
        XBee.println(buf);
    else if(sel == 1){// send to SD Card
        SendToSD(buf);
    }
}

static uint8_t get_stringcmd() //read the xbee buffer, return a flag if it's time to execute.
{
    char c;
    bool flag = 0;
    while(XBee.available()){ //while there's a chacter in the Xbee buffer, read.  add buffer to the command global variable.  if end of line character, set flag to 1.
        c = XBee.read();
        Serial.write(c); //echo to the screen for debugging
        if(g_cmdIndex == CMD_BUF_SIZE){//if index is greater than array size
            g_cmdIndex = CMD_BUF_SIZE - 1;
        }  
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
    if(strcmp(cmd, "SA")==0){ //"SA" sets sampling status.  SA = 1 is sampling on.
        log_pck.isSampling = *(int*)val;  
        digitalWrite(MOTOR_ENABLE_PIN, *(int*)val);                   //output val to motor enable relay.
        Serial.print("issampling(SA): ");
        Serial.println(log_pck.isSampling);                
        
    } else if (strcmp(cmd, "MC")==0){ //"MC" writes the motor command, from 0-255 (for digital pot) and from 0-4095 (for dac), manually.  Motor control must be in mode 1 to matter.
        log_pck.motorcommand = *(int*)val;
        CHECK_MOTOR_CMD(&log_pck.motorcommand);
        dac.setVoltage(log_pck.motorcommand, false);
//        Serial.print("mode command(MC): ");
//        Serial.println(log_pck.motorcommand);
        
    } else if (strcmp(cmd, "MD")==0){ //"MD" sets the mode for motor speed control. 0 = auto, PID feedback, 1 = manual control using "MC" command to set speed.
        g_motor_control_mode = *(int*)val;
        log_pck.mode = g_motor_control_mode;
        Serial.print("mode changed(MD): ");
        Serial.println(g_motor_control_mode);
    }
  
    return 0;  
}
/**
 * parse sring command and execute
*/
static void parse_stringcmd(char* buf)
{
    char cmd[2]; //cmd is at most 2 characters, ie, SA
    int i=0;

    cmd[0] = buf[0];
    cmd[1] = buf[1];
    cmd[2] = '\0';
   
    i = atoi(&buf[3]);
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

void update_sensors()        //update values from all sensors.  
{
    double error;
    float  iso_nozzle_diameter = .0062f;   // isokinetic nozzle diameter in meters
    g_tempC_inline = (ina219Temp.getCurrent_mA()-4)/16*100; //get 4-20ma signal and convert to 0-100C scale.  No ring buffer for this value
    g_RH_inline = (ina219RH.getCurrent_mA()-4)/16*100; //get 4-20ma signal and convert to 0-100% scale.  No ring buffer for this value
    g_tempC_enclosure = htu.readTemperature(); //no ring buffer necessary
    g_RH_enclosure = htu.readHumidity(); //no ring buffer necessary
    
    /**<
     * updraft velocity read:
     * adc value * multiplier for gain 1 of ads1115 * mph coefficient
     * ADS1115 @ +/- 4.096V gain (16-bit results)
     * Output is in mph 
     */
    //double updraft_v = ads.readADC_Differential_0_1() * 0.125f * 0.018; //[adc value * Gain 1 coeff * m/s coeff], velocity in m/s
    double updraft_v = 12.0;
    /**< 
     * inline flow read: 
     * adc value * multiplier for gain 1 of ads1015 * 2 [because voltage into adc has doubled from 5v to 10v]
     * Output is in mV but converted to L/min 
     */
    double inline_f  = MAP(ads_i.readADC_SingleEnded(1) * 0.1875f * 2, 0.0, 10000.0, 0.0, 200.0)  //read TSI inline flow meter.  Gain 2/3 coeff. Map 0-10V to 0-200 lpm. Multiply original voltage by 2 because of voltage divider.
                                  * (273.15 + g_tempC_inline) / (273.15 + 21.11); //units = liters/min

    /**store velocity and flow in ring buffer.*/
    if(g_buffer_index == BUF_SIZE){
        g_buffer_index = 0;  
    }
    
    log_pck.updraftVel      = movingAverage(g_updraftVelBuf, &g_updraft_sum, g_buffer_index,BUF_SIZE, updraft_v);
    log_pck.inlineFlow      = movingAverage(g_inlineFlowBuf, &g_inline_sum, g_buffer_index,BUF_SIZE, inline_f );
    
    /*calulate nozzle velocity*/
    log_pck.nozzleVel       = log_pck.inlineFlow /60/1000/ (5*(3.1415*pow((iso_nozzle_diameter/2),2))); //units = m/s
    
    log_pck.tempC           = g_tempC_inline;
    log_pck.rh              = g_RH_inline;

    log_pck.encl_tempC           = g_tempC_enclosure;
    log_pck.encl_rh              = g_RH_enclosure;

    /*P control for motor*/
    error        = log_pck.nozzleVel - log_pck.updraftVel; //units = m/s

    if(g_motor_control_mode == 0) { //if in auto PID control mode...
#define KP -10
      log_pck.motorcommand = log_pck.motorcommand + KP*error;  //for digital pot: recall that 255 = motor off, 0 = full speed. positive error means motor is spinning too fast.
    }
    else if (g_motor_control_mode == 1) { //if in mode 1, taking manual commands via serial for speed setting.  Don't recalculate motor command in feedback.
    }


    CHECK_MOTOR_CMD(&log_pck.motorcommand);
//    if(log_pck.motorcommand > 4095) log_pck.motorcommand = 4095;
//    else if(log_pck.motorcommand < 0) log_pck.motorcommand = 0;
    
    if(log_pck.isSampling){//if we want to sample...
        execute_cmd(&log_pck.motorcommand, "MC"); //send the current motor command value to the i2c potentiometer
        digitalWrite(MOTOR_ENABLE_PIN, HIGH); // and turn on solenoid using rugged shield (make sure it's on)
    }
    else{//make sure motors are off
        log_pck.motorcommand    = 0; //for digital pot, 255 is lowest speed, for dac, 0 is lowest speed
        digitalWrite(MOTOR_ENABLE_PIN, LOW); //verify motor solenoid is off
    }
    g_buffer_index++;
}

void tests()
{
//    while(1);
}
void loop()
{
    if(XBee.available() > 0){ //if serial buffer received command //
        if(get_stringcmd()){//if we received newline character, execute what's in the command buffer so far.
            parse_stringcmd(g_cmdBuffer);  //parse and execute command
            memset(g_cmdBuffer, 0, sizeof(g_cmdBuffer)); //zero out buffer
        }
    }
    if(g_update_flag){//if flag is set
      
        g_update_flag = 0;
        update_sensors();
        
        if(g_Scycles >= PERIOD_SENDDATA){//send wirelessly
            g_Scycles = 0;
            send_log(&log_pck, 0);      
        }
        if(g_Lcycles >= PERIOD_LOGDATA){//log to sd card 
            g_Lcycles = 0;
            send_log(&log_pck, 1);
        }
    }
//    tests(); //function used to test other functions.
}
