#include <Wire.h>
#include <Adafruit_INA219.h>

//This version uses a 10 ohm resistor, not the standard resistor!  You must modify the INA219 board.
//This version also uses a custom version of the INA219 library with support for the 10 ohm resistor.  Get the library from GitHub

Adafruit_INA219 AmbTemp(0x40); // default address, no jumper pads soldered
Adafruit_INA219 AmbRH(0x45); // both jumper pads soldered



void setup(void) 
{
  Serial.begin(9600);
  while (!Serial) {
      // will pause Zero, Leonardo, etc until serial console opens
      delay(1);
  }

  uint32_t currentFrequency;
    
  Serial.println("Hello!");
  
  // Initialize the INA219.
  // By default the initialization will use the largest range (32V, 2A).  However
  // you can call a setCalibration function to change this range (see comments).
  AmbTemp.begin();
  AmbRH.begin();



  AmbTemp.setCalibration_32V_20mA(); //custom library required for this line
  AmbRH.setCalibration_32V_20mA();


  Serial.println("Measuring voltage and current with INA219 ...");
}

void loop(void) 
{
  float current_mAT_amb = 0;
  float current_mARH_amb = 0;

  current_mAT_amb = AmbTemp.getCurrent_mA();
  current_mARH_amb = AmbRH.getCurrent_mA();

  float RHpct_amb = (current_mARH_amb-4)/16*100; //0-100% RH range
  float TempC_amb = (current_mAT_amb-4)/16*100-40; //temperature range for HMS110 is from -40C to 160C
  
  
  Serial.print("Current T:       "); Serial.print(current_mAT_amb); Serial.println(" mA");
  Serial.print("Temp C:         "); Serial.print(TempC_amb);
  Serial.println("");

  Serial.print("Current RH:       "); Serial.print(current_mARH_amb); Serial.println(" mA");
  Serial.print("Relative Humidity:         "); Serial.print(RHpct_amb);
  Serial.println("");
  delay(2000);
}
