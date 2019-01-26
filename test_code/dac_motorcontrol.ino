#include <Wire.h>
#include <Adafruit_MCP4725.h>

Adafruit_MCP4725 dac;

void setup(void) {
  Serial.begin(9600);
  Serial.println("Hello!");

  // For Adafruit MCP4725A1 the address is 0x62 (default) or 0x63 (ADDR pin tied to VCC)
  // For MCP4725A0 the address is 0x60 or 0x61
  // For MCP4725A2 the address is 0x64 or 0x65
  dac.begin(0x62);
    
}
     
#define MAP(x, a, b, c, d) \
    ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
       __typeof__ (c) _c = (c); \
       __typeof__ (d) _d = (d); \
       __typeof__ (x) _x = (x); \
    ((_x - _a) / (_b - _a) * (_d - _c) + _c); })


void loop(void) {
    uint32_t counter;
    // Run through the full 12-bit scale for a triangle wave
    for (counter = 0; counter < 4095; counter++)
    {
      dac.setVoltage(counter, false);
      Serial.println(counter);
      delay(10);
    }
    for (counter = 4095; counter > 0; counter--)
    {
      dac.setVoltage(counter, false);
        Serial.println(counter);
        delay(10);
    }
//    double VOUT = 5.0;
//    Serial.println(MAP(VOUT, 0.0, 5.0, 0.0, 4095.0));
//    dac.setVoltage(MAP(VOUT, 0.0, 5.0, 0.0, 4095.0), false);
}
