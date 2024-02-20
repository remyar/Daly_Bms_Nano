#include <Arduino.h>
#include "daly.h"

DalyBms bms(2, 3);

void setup()
{
  Serial.begin(9600);
  bms.Init(); // init the bms driver
}

void loop()
{
  bms.loop();
}
