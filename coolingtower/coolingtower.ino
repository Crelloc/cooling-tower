/*
 * * Cooling Tower Project
 * * Author: Thomas Turner, thomastdt@gmail.com
 * * Last Modified: 08-28-18
*/

#include <SPI.h>
#include <SD.h>
//#include "RTClib.h"

#define BUF_SIZE 6

static File           dataFile;
static volatile int   g_cycles        = 0;             /**< # of cycles for timer1 */
static volatile bool  log_flag        = 0;             /**< flag to log to SD card and send data to another arduino via Serial*/
static volatile bool  request_flag    = 0;             /**< flag to parse data sent by another arduino*/
static volatile bool  update_flag     = 0;             /**< flag to calculate velocity, flow, and motor command*/
static volatile float buffer[2]       = {0.0f, 0.0f};  /**< buffer[0] = analog read for updraft velocity, buffer[1] = analog read for inline flow*/
static volatile bool  isSampling_flag = 0;

void setup() 
{

    /** Open serial communications and wait for port to open: */
    while (!Serial) {
        ; /**< wait for serial port to connect. Needed for native USB port only */
    }
    Serial.begin(9600);

    Serial.print("\nInitializing SD card...");
    // make sure that the default chip select pin is set to
    // output, even if you don't use it:
    pinMode(4, OUTPUT); /**Wireless SD shield uses pin 4 as chipselect*/
    // we'll use the initialization code from the utility libraries
    // since we're just testing if the card is working!
    if (!SD.begin(4)) {
        Serial.println("initialization failed. Things to check:");
        Serial.println("* is a card inserted?");
        Serial.println("* is your wiring correct?");
        Serial.println("* did you change the chipSelect pin to match your shield or module?");
       // return;
    } else {
        Serial.println("Wiring is correct and a card is present.");
    }

    /** initialize timer1 - 16 bit (65536) */
    noInterrupts();                            // disable all interrupts
    TCCR1A  = 0;
    TCCR1B  = 0;
    TCNT1   = 49911;                           // preload timer
    TCCR1B |= ((1 << CS12)| (1 << CS10)) ;     // prescaler 
    TIMSK1 |= (1 << TOIE1);                    // enable timer overflow interrupt
    interrupts();                              // enable all interrupts

}

/**Timer frequency: 1 cycle per second*/
ISR(TIMER1_OVF_vect)        
{
//#define PERIOD_THRESHOLD1 1   /** Every 1 cycle read analog sensors in buffer and set flag to calculate velocity and flow, and to update motor speed*/
#define PERIOD_THRESHOLD2 10  /** Every 10 cycles set flag to log data and send over Serial*/
    TCNT1 = 49911;

    //buffer[0] = analogreadfunctiontobecoded; //Read analog voltage in mV using ADC. Updraft Vel
    //buffer[1] = analogreadfunctiontobecoded; //Read analog voltage in mV using ADC. Expect 0-10VDC signal. Inline flow

    g_cycles++;               /** number of seconds elapsed*/
    update_flag = true;
    
    if(PERIOD_THRESHOLD2 == g_cycles){
        log_flag    = true;        
        g_cycles = 0;
    }
}

/**will log to sd card*/
static int log_info() 
{
    File dataFile = SD.open("datalog.txt", FILE_WRITE);

    // if the file is available, write to it:
    if (dataFile) {
        dataFile.println("test string");
        dataFile.close();
        // print to the serial port too:
        Serial.println("test string");
    } else {
        Serial.println("error opening datalog.txt");
        return -1;
    }
    return 0;
}

/**send via wireless signal*/
static int send_pkt() 
{
    return 0;
}

void loop()
{

    if(log_flag){
      
        log_flag = 0;
        log_info();
        send_pkt();  
    }
    if(request_flag){ /**< flag to parse data sent by another arduino.  This flag may change to using built in Serial flag*/
        request_flag = 0;
        
    }              
    if(update_flag){
        update_flag = 0;
          
    }

}
