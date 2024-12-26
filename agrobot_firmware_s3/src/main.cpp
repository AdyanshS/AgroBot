
#include "Arduino.h"
#include "sharp_ir_driver.hpp"

#define ir_pin 3

void setup()
{
  setupSharpIRsensor();

  // pinMode()

  Serial.begin(115200);
}
void loop()
{
  Serial.println(getDistanceinCM(SharpIR1));

  delay(20); // read data every 2 seconds
}
