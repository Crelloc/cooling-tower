/*
 * * Cooling Tower Project
 * * Author: Thomas Turner, thomastdt@gmail.com
 * * Last Modified: 09-11-18
*/

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>

Adafruit_ADS1115 ads;
//#include "RTClib.h"


typedef struct Log_Pck_Struct {
    bool isSampling;
    float rh;
    float tempC;
    double updraftVel;
    double inlineFlow;
    float nozzleVel;
    int motorcommand;
} Log_Pck_Struct;

#define BUF_SIZE 6

/**Global Variables*/
static char g_cmdBuffer[32]                   = {};
static char g_cmdIndex                        =  0;
static double g_updraftVelBuf[BUF_SIZE]       = {};
static double g_inlineFlowBuf[BUF_SIZE]       = {};
static Log_Pck_Struct log_pck                 = {};

/**Variables used by interrupts*/
static volatile int g_updraft;
static volatile int g_inlineFlow;
static volatile int g_tempC;

static volatile uint8_t g_buffer_index        = 0;
static volatile int   g_cycles                = 0;  /**< # of cycles for timer1 */
static volatile bool  g_update_flag           = 0;  /**< flag to calculate velocity, flow, and motor command*/
static float                           g_sum  = 0;

void setup() 
{

                                                 
    while (!Serial) {                            /** Open serial communications and wait for port to open: */
        ;                                        /**< wait for serial port to connect. Needed for native USB port only */
    }
    Serial.begin(9600);

    pinMode(LED_BUILTIN, OUTPUT);                //temporary digital output to motor pin.
    Serial.print("\nInitializing SD card...");
                                                 // make sure that the default chip select pin is set to
                                                 // output, even if you don't use it:
    pinMode(4, OUTPUT);                          /**Wireless SD shield uses pin 4 as chipselect*/
                                                 // we'll use the initialization code from the utility libraries
                                                 // since we're just testing if the card is working!
    if (!SD.begin(4)) {
        Serial.println("initialization failed. Things to check:");
        Serial.println("* is a card inserted?");
        Serial.println("* is your wiring correct?");
        Serial.println("* did you change the chipSelect pin to match your shield or module?");

    } else {
        Serial.println("Wiring is correct and a card is present.");
    }

    ads.setGain(GAIN_ONE);
    ads.begin();

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
                           
#define PERIOD_LOGDATA 2  /** Every 10 cycles set flag to log data and send over Serial*/
    TCNT1 = 49911;

    g_updraft   = ads.readADC_Differential_0_1();
    //g_inlineFlow = analogreadfunctiontobecoded; //Read analog voltage in mV using ADC. Expect 0-10VDC signal. Inline flow
    //g_tempC      = analogreadfunctiontobecoded;
 
    ++g_cycles;               /** number of seconds elapsed*/
    g_update_flag = 1;
   
}

//static void sprintf_f(float fval, char *c)
//{
//    char *tmpSign = (fval < 0) ? "-"   : "";
//    float tmpVal  = (fval < 0) ? -fval : fval;
//
//    int tmpInt1   = tmpVal;
//    float tmpFrac = tmpVal - tmpInt1;
//    int tmpInt2 = trunc(tmpFrac * 10000);
//
//    sprintf(c, "fval = %s%d.%04d", tmpSign, tmpInt1, tmpInt2);
//   
//}

/**will log to sd card*/
static int log_info(Log_Pck_Struct *pck) 
{
    int ret = 0;
    File dataFile = SD.open("datalog.txt", FILE_WRITE);

    // if the file is available, write to it:
    if (dataFile) {
        dataFile.print("Sampling = ");
        dataFile.print(pck->isSampling);
        dataFile.print(", rh (%)= ");
        dataFile.print(pck->rh, 4);
        dataFile.print(", tempC = ");
        dataFile.print(pck->tempC, 2);
        dataFile.print(", v_updraft = ");
        dataFile.print(pck->updraftVel, 4);
        dataFile.print(", inlineFlow = ");
        dataFile.print(pck->inlineFlow, 4);
        dataFile.print(", v_nozzle = ");
        dataFile.print(pck->nozzleVel, 4);
        dataFile.print(", motorcmd = ");
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
    Serial.print("Sampling = ");
    Serial.print(pck->isSampling);
    Serial.print(", rh = (%)");
    Serial.print(pck->rh, 4);
    Serial.print(", tempC = ");
    Serial.print(pck->tempC, 2);
    Serial.print(", v_updraft = ");
    Serial.print(pck->updraftVel, 4);
    Serial.print(", inlineFlow = ");
    Serial.print(pck->inlineFlow, 4);
    Serial.print(", v_nozzle = ");
    Serial.print(pck->nozzleVel, 4);
    Serial.print(", motorcmd = ");
    Serial.println(pck->motorcommand);
    
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

static uint8_t get_stringcmd()
{
    char c;
    bool flag = 0;
    while(Serial.available()){
        c = Serial.read();
        if(g_cmdIndex == 64){/*error check*/}  
        if(c == '\r' || c == '\n'){
            g_cmdBuffer[g_cmdIndex] = '\0';
            g_cmdIndex = 0;
            flag = 1;
            break;
        } else {
            g_cmdBuffer[g_cmdIndex] = c;
        }
        g_cmdIndex++;
         
    }
    return flag;
}

static int execute_cmd(void* val, char const* cmd)
{
    if(strcmp(cmd, "S")==0){
        log_pck.isSampling = *(int*)val;
        //send value to digital pin?
        
    } else if(strcmp(cmd, "U")==0){
        int tval = *(int*)val;
        //send value to digital pin? 
    }
    return 0;  
}

void parse_stringcmd(char* buf)
{
    char cmd[2] = {};
    int i = -1;
    sscanf(buf, "%s %d", cmd, &i);
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

#define MAP(x, a, b, c, d) ((x - a) / (b - a) * (d - c) + c)

void update_sensors()        
{
    float MULTIPLIER        = 0.125f;  /**< ADS115 @ +/- 4.096V gain (16-bit results)*/
    double MPH_COEFFICIENT  = 0.04025; 
    uint16_t gainfactor     = 1;       // 1 is placeholder
    int iso_nozzle_diameter = 2;       // 2 is place holder
    float tempC             = MAP(g_tempC, 4.0f, 20.0f, 0.0f, 100.0f);
    double updraft_v        = g_updraft * MULTIPLIER * MPH_COEFFICIENT ; //velocity in mph
    double inline_f         = MAP(g_inlineFlow, 0.0, 10000.0, 0.0, 200.0)
                                  * (273.15 + tempC) / (273.15 + 21.11);
    
    /**store vel and flow in ring buffer.
    */
    if(g_buffer_index == BUF_SIZE){
        g_buffer_index = 0;  
    }
    
    log_pck.updraftVel      = movingAverage(g_updraftVelBuf, &g_sum, g_buffer_index,BUF_SIZE, updraft_v);
    log_pck.inlineFlow      = movingAverage(g_inlineFlowBuf, &g_sum, g_buffer_index,BUF_SIZE, inline_f );
    log_pck.nozzleVel       = log_pck.inlineFlow / 5*(3.1415*pow((iso_nozzle_diameter>>1),2));
    log_pck.tempC           = tempC;
    log_pck.rh              = tempC;

    g_buffer_index++;
    
    if(log_pck.isSampling){
        log_pck.motorcommand   += (int)(gainfactor*(log_pck.nozzleVel-log_pck.updraftVel));
        //output command to motor
        execute_cmd(&log_pck.motorcommand, "U");
    } else{
        log_pck.motorcommand    = 0;
        //output command to motor
        execute_cmd(&log_pck.motorcommand, "S");
    }

}

void tests()
{
    while(1);
}


void loop()
{

    while(!g_update_flag && Serial.available() == 0 );

    if(Serial.available() > 0){
        if(get_stringcmd()){
//            Serial.print("cmd: ");
//            Serial.println(g_cmdBuffer);
            parse_stringcmd(g_cmdBuffer);  
        }
    }
    if(g_update_flag){
      
        g_update_flag = 0;
        update_sensors();

        if(g_cycles >= PERIOD_LOGDATA){
          
                g_cycles = 0;
                log_info(&log_pck);
                send_pkt(&log_pck);
        }

    }
//      tests();
}
