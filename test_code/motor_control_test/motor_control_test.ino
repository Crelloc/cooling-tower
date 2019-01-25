/* Test code to check functionality of digitial potentiometer, 
 * flow enable/disable relay, and speed control
 * 
 * Last edited 1/25/19 CDW
  */

#include <Wire.h>
#include <Adafruit_ADS1015.h>

int motorCommand = 0;

void setup() {
  
  Serial.begin(9600);
  Wire.begin(); // join i2c bus (address optional for master)
  pinMode(6, OUTPUT); //motor enable pin
  digitalWrite(6, HIGH);    
  

}

void loop() {
  //Serial.println("motor_on");
  //digitalWrite(6, HIGH);
  
  //Digital Potentiometer:
  Wire.beginTransmission(44); // 
  // device address is specified in datasheet
  Wire.write(byte(16));            // sends instruction byte
  Wire.write(motorCommand);             // sends potentiometer value byte
  Wire.endTransmission();     // stop transmitting
  Serial.print("Motor command = ");
  Serial.println(motorCommand);
  delay(1000);

  motorCommand += 5;
  if (motorCommand > 255) motorCommand = 0;

}
