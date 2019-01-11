#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219Temp(0x44);
Adafruit_INA219 ina219RH(0x41);



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
  ina219Temp.begin();
  ina219RH.begin();

  // To use a slightly lower 32V, 1A range (higher precision on amps):
  //ina219.setCalibration_32V_1A();
  ina219Temp.setCalibration_32V_20mA();
    ina219RH.setCalibration_32V_20mA();

  // Or to use a lower 16V, 400mA range (higher precision on volts and amps):
  //ina219.setCalibration_16V_400mA();

  Serial.println("Measuring voltage and current with INA219 ...");
}

void loop(void) 
{
  float current_mAT = 0;
  float current_mARH = 0;

  current_mAT = ina219Temp.getCurrent_mA();
  current_mARH = ina219RH.getCurrent_mA();

  float RHpct = (current_mARH-4)/16*100; //0-100% RH range
  float TempC = (current_mAT-4)/16*100; //0-70C range, but scaled to 0-100
  
  
  Serial.print("Current T:       "); Serial.print(current_mAT); Serial.println(" mA");
  Serial.print("Temp C:         "); Serial.print(TempC);
  Serial.println("");

  Serial.print("Current RH:       "); Serial.print(current_mARH); Serial.println(" mA");
  Serial.print("Relative Humidity:         "); Serial.print(RHpct);
  Serial.println("");
  delay(2000);
}
