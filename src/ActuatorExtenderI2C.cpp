#include "ActuatorExtenderI2C.h"
    
ActuatorExtenderI2C::ActuatorExtenderI2C(uint8_t pca9685Addr, uint8_t ads1015Addr, bool servo0Connected, bool servo1Connected, bool oeConnected, bool alertConnected, bool emsConnected, uint32_t oscillatorFrequency):
Adafruit_PWMServoDriver(pca9685Addr), Adafruit_ADS1015(ads1015Addr)
{
  this->pca9685Addr = pca9685Addr;
  this->ads1015Addr = ads1015Addr;

  this->servo0Connected = servo0Connected;
  this->servo1Connected = servo1Connected;
  this->oeConnected = oeConnected;
  this->alertConnected = alertConnected;
  this->emsConnected = emsConnected;

  this->oscillatorFrequency = oscillatorFrequency;
}

ActuatorExtenderI2C::~ActuatorExtenderI2C()
{
 if(servo0Connected)
 {
  delete servo0;
 }
 
 if(servo1Connected)
 {
  delete servo1; 
 }
}

bool ActuatorExtenderI2C::begin()
{
  bool success = false;
  byte pcaAck, adsAck;
  Wire.begin();
  Wire.beginTransmission(pca9685Addr);
  pcaAck = Wire.endTransmission();
  Wire.beginTransmission(ads1015Addr);
  adsAck = Wire.endTransmission();

  if(pcaAck == 0 && adsAck == 0)
  {
    success = true;
    
    ActuatorExtenderI2C::Adafruit_ADS1015::begin();
    
    ActuatorExtenderI2C::Adafruit_PWMServoDriver::begin();
    setOscillatorFrequency(oscillatorFrequency);
    
    configureForPowertrain();
  }

  if(oeConnected)
  {
    pinMode(mcuPinmap.oe, OUTPUT);
    enableOutputs(true);
  }

  if(alertConnected)
  {
    pinMode(mcuPinmap.alert, INPUT);
  }

  if(emsConnected)
  {
    pinMode(mcuPinmap.ems, INPUT);
  }

  if(servo0Connected)
  {
    servo0 = new Servo();
    servo0->attach(mcuPinmap.servo0);
  }


  if(servo1Connected)
  {
    servo1 = new Servo();
    servo1->attach(mcuPinmap.servo1);
  }
  
  return success;  
}

void ActuatorExtenderI2C::enableOutputs(bool enable)
{
  if(oeConnected)
  {
    pinMode(mcuPinmap.oe, enable);
  }
}

void ActuatorExtenderI2C::configureForPowertrain()
{
  setPWMFreq(1600.0);
}

void ActuatorExtenderI2C::configureForServos()
{
  setPWMFreq(50.0);
}

void ActuatorExtenderI2C::setLed(uint8_t number, bool state)
{
  if(number < 4 && number >= 0)
  {
    uint8_t pin;

    switch(number)
    {
      case 0: pin = pcaPinmap.led0; break;
      case 1: pin = pcaPinmap.led1; break;
      case 2: pin = pcaPinmap.led2; break;
      case 3: pin = pcaPinmap.led3; break;
    }

    if(state)
    {
      setPin(pin, PCA_HIGH);
    }
    else
    {
      setPin(pin, PCA_LOW);
    }
  }
}

void ActuatorExtenderI2C::setLeds(uint8_t value)
{
  setLed(0, value & 0b00000001);
  setLed(1, value & 0b00000010);
  setLed(2, value & 0b00000100);
  setLed(3, value & 0b00001000);
}

void ActuatorExtenderI2C::enablePowertrain(bool left, bool right)
{
  enableLeftMotor(left);
  enableRightMotor(right);  
}

void ActuatorExtenderI2C::enableLeftMotor(bool enable)
{
  if(enable)
  {
    setPin(pcaPinmap.inh12, PCA_HIGH);
  }
  else
  {
    setPin(pcaPinmap.inh12, PCA_LOW);
  }
}

void ActuatorExtenderI2C::enableRightMotor(bool enable)
{
  if(enable)
  {
    setPin(pcaPinmap.inh34, PCA_HIGH);
  }
  else
  {
    setPin(pcaPinmap.inh34, PCA_LOW);
  }
}

void ActuatorExtenderI2C::setDutycycles(short left, short right)
{
  setDutycycleLeft(left);
  setDutycycleRight(right);  
}

void ActuatorExtenderI2C::setDutycycleLeft(short dutycycle)
{
	static short last = 0xffff;

	if(last != dutycycle)
	{
	  bool forward = true;

		if(dutycycle < 0)
		{
		  forward = false;
		  dutycycle = -dutycycle;
		}

		uint16_t counter = (uint16_t)(4096 * (dutycycle / 100.0));

		if(counter > 4095)
		{
		  counter = 4095;  
		}
		
		if(forward)
		{
		  setPin(pcaPinmap.pwm1, counter);
		  setPin(pcaPinmap.pwm2, PCA_LOW);
		}
		else
		{ 
		  setPin(pcaPinmap.pwm1, PCA_LOW);
		  setPin(pcaPinmap.pwm2, counter);    
		}

		last = dutycycle;
	}
}

void ActuatorExtenderI2C::setDutycycleRight(short dutycycle)
{
	static short last = 0xffff;

	if(last != dutycycle)
	{
	  bool forward = true;
		if(dutycycle < 0)
		{
		  forward = false;
		  dutycycle = -dutycycle;
		}

		uint16_t counter = (uint16_t)(4096 * (dutycycle / 100.0));

		if(counter > 4095)
		{
		  counter = 4095;  
		}
		
		if(forward)
		{
		  setPin(pcaPinmap.pwm3, counter);
		  setPin(pcaPinmap.pwm4, PCA_LOW);
		}
		else
		{
		  setPin(pcaPinmap.pwm3, PCA_LOW);
		  setPin(pcaPinmap.pwm4, counter);    
		}  

		last = dutycycle;
	}
}

void ActuatorExtenderI2C::breakPowertrain()
{
  breakLeftMotor();
  breakRightMotor();
}

void ActuatorExtenderI2C::breakLeftMotor()
{
  setPin(pcaPinmap.pwm1, PCA_HIGH);
  setPin(pcaPinmap.pwm2, PCA_HIGH);
}

void ActuatorExtenderI2C::breakRightMotor()
{
  setPin(pcaPinmap.pwm3, PCA_HIGH);
  setPin(pcaPinmap.pwm4, PCA_HIGH);  
}

void ActuatorExtenderI2C::setServos(short servo0, short servo1)
{
  setServo0(servo0);
  setServo1(servo1);
}

void ActuatorExtenderI2C::setServo0(short pos)
{
	static short last = 0xffff;

	if(last != pos)
	{
		if(pos < 0)
		{
		  pos = 0;
		}
		else if(pos > 180)
		{
		  pos = 180;
		}
		
		if(servo0Connected)
		{
		  setPin(pcaPinmap.servo0, SERVO_MIN + (uint16_t)((SERVO_MAX-SERVO_MIN) * (pos / 180.0)));
		}
		else
		{
		  servo0->write(pos);
		}
		last = pos;
	}
}

void ActuatorExtenderI2C::setServo1(short pos)
{
	static short last = 0xffff;

	if(last != pos)
	{
		if(pos < 0)
		{
		  pos = 0;
		}
		else if(pos > 180)
		{
		  pos = 180;
		}
		
		if(servo0Connected)
		{
		  setPin(pcaPinmap.servo1, SERVO_MIN + (uint16_t)((SERVO_MAX-SERVO_MIN) * (pos / 180.0)));
		}
		else
		{
		  servo0->write(pos);
		}  
		last = pos;
	}
}

bool ActuatorExtenderI2C::isAlert()
{
  if(alertConnected)
  {
    return digitalRead(mcuPinmap.alert);
  }
  else
  {
    return false;
  }
}

bool ActuatorExtenderI2C::isEmergencyStop()
{
  if(emsConnected)
  {
    return !digitalRead(mcuPinmap.ems);
  }
  else
  {
    return false;
  }
}

uint16_t ActuatorExtenderI2C::measureBemfLeft(int delayMs)
{
  enableLeftMotor(false);
  delay(delayMs);
  uint16_t value = readADC_SingleEnded(adsPinmap.bemf12);
  enableLeftMotor();
  return value;
}

uint16_t ActuatorExtenderI2C::measureBemfRight(int delayMs)
{
  enableRightMotor(false);
  delay(delayMs);
  uint16_t value = readADC_SingleEnded(adsPinmap.bemf34);
  enableRightMotor();
  return value;
}

void ActuatorExtenderI2C::measureBemfs(uint16_t *left, uint16_t *right, int delayMs)
{
  enableLeftMotor(false);
  enableRightMotor(false);
  delay(delayMs);
  *left = readADC_SingleEnded(adsPinmap.bemf12);
  *right = readADC_SingleEnded(adsPinmap.bemf34);
  enableLeftMotor();
  enableRightMotor();
}

uint16_t ActuatorExtenderI2C::measureCurrentLeft()
{
	return readADC_SingleEnded(adsPinmap.is12);
}

uint16_t ActuatorExtenderI2C::measureCurrentRight()
{
	return readADC_SingleEnded(adsPinmap.is34);
}
