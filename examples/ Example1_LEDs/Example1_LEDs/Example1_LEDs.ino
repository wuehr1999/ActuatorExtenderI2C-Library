/******************************************************************************
  Simple LED counter with user LEDs
  
  Original Creation Date: 05.01.2021

  Hardware Connections:
  ActuatorExtenderI2C over I2C

******************************************************************************/
#include <ActuatorExtenderI2C.h>

ActuatorExtenderI2C extender;
void setup() {
  extender.begin();
}

void loop() {
  for(uint8_t i = 0; i < 0xf; i++)
  {
    extender.setLeds(i);
    delay(500);
  }
}
