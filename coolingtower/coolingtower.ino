/*
 * * Cooling Tower Project
 * * Author: Thomas Turner, thomastdt@gmail.com
 * * Last Modified: 08-28-18
*/

#include <SPI.h>
#include <SD.h>
//#include "RTClib.h"

File        dataFile;
const char  FILE_PATH[]   = "datalog.txt";

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

}

void loop() {

}
