/******************************************************************************
  Test both powertrain motors
  
  Original Creation Date: 05.01.2021

  Hardware Connections:
  ActuatorExtenderI2C over I2C
  Motors connected
  Emergency stop button input connected

******************************************************************************/
#include <ActuatorExtenderI2C.h>

ActuatorExtenderI2C extender;
void setup() {
  extender.begin();
  extender.enablePowertrain();
}

void loop() {
  for(short i = -100; i < 100; i++)
  {
    extender.setDutycycles(i, i);
    delay(250);
  }
  for(short i = 100; i > -100; i--)
  {
    extender.setDutycycles(i, i);
    delay(250);
  } 
}
