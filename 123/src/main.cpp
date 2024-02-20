#include <Arduino.h>
#include <daly.h>

DalyBms bms = DalyBms();

void setup()
{
  Serial.begin(115200);

  pinMode(8 , INPUT_PULLUP);
  pinMode(9 , INPUT_PULLUP);
  bms.Init(); // init the bms driver
}

void loop()
{
  bms.loop();
}
