#ifndef ACTUATOREXTENDERI2C_H
#define ACTUATOREXTENDERI2C_H

#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_ADS1015.h>
#include <Wire.h>
#include <Servo.h>

class ActuatorExtenderI2C : public Adafruit_PWMServoDriver, public Adafruit_ADS1015
{
  public:

    /*!
     * @brief Constructor
     * @param pca9685Addr Address of PCA9685 I2C PWM driver
     * @param ads1015Addr Address of ADS1015 I2C ADC 
     * @param servo0Connected, servo1Connected Servos directly connected to MCU and not over PCA9685 (check R12, R13, R14 and R55)
     * @param oeConnected PCA9685 Output enable pin connected to pin 4 of MCU (JP8)
     * @param alertConnected ADS1015 alert interrupt connected to pin 3 of MCU (JP9)
     * @param emsConnected Emergency stop pin (LOW active) connected to pin 5 of MCU (JP10)
     * @param oscillatorFrequency PCA9685 oscillator base frequeny in Hz
     */
    ActuatorExtenderI2C(uint8_t pca9685Addr = 0x40, uint8_t ads1015Addr = 0x48, bool servo0Connected = false, bool servo1Connected = false, bool oeConnected = true, bool alertConnected = true, bool emsConnected = true, uint32_t oscillatorFrequency = 25000000);
  
    /*!
     * @brief Destructor
     */
    ~ActuatorExtenderI2C();

    /*!
     * @brief Initialization function
     * @retval Initialization successfull
     */
    bool begin();

    /*!
     * @brief If PCA9685 OE is connected to pin 4 of MCU, the outputs can be enabled/ disabled (JP8)
     * @param enable Enable state
     */
    void enableOutputs(bool enable = true);

    /*!
     * @brief Set single LED
     * @param number LED number
     * @param state LED on/ off
     */
    void setLed(uint8_t number, bool state);

    /*!
     * @brief Show binary value on all 4 LEDs
     * @param value Value
     */
    void setLeds(uint8_t value);

    /*!
     * @brief Configures PCA9685 PWM frequency to 50 Hz for servo control
     */
    void configureForServos();

    /*!
     * @brief Configures PCA9685 PWM frequency to 1.6 kHz for powertrain control
     */
    void configureForPowertrain();

    /*!
     * @brief Enable both motors
     * @param left Enable left motor
     * @param right Enable right motor
     */
    void enablePowertrain(bool left = true, bool right = true);

    /*!
     * @brief Enable left motor
     * @param enable Enable state
     */
    void enableLeftMotor(bool enable = true);

    /*!
     * @brief Enable right motor
     * @param enable Enable state
     */
    void enableRightMotor(bool enable = true);

    /*!
     * @brief Set dutycycles of both motors
     * @param left Left dutycycle from -100 to 100 (%)
     * @param right Right dutycycle from -100 to 100 (%)
     */
    void setDutycycles(short left, short right);

    /*!
     * @brief Set dutycycle of left motor
     * @param dutycycle Dutycycle from -100 to 100 (%)
     */
    void setDutycycleLeft(short dutycycle);

    /*!
     * @brief Set dutycycle of right motor
     * @param dutycycle Dutycycle from -100 to 100 (%)
     */
    void setDutycycleRight(short dutycycle);

    /*!
     * @brief Break both motors
     */
    void breakPowertrain();

    /*! 
     * @brief Break left motor
     */
    void breakLeftMotor();

    /*!
     * @brief Break right motor
     */
    void breakRightMotor();

    /*!
     * @brief Set positions of both servos
     * @param servo0 Position from 0 to 180 (deg)
     */
    void setServos(short servo0, short servo1);

    /*!
     * @brief Set position of Servo 0
     * @param pos Position from 0 to 180 (deg)
     */
    void setServo0(short pos);

    /*!
     * @brief Set position of Servo 1
     * @param pos Position from 0 to 180 (deg)
     */
    void setServo1(short pos);

    /*!
     * @brief Check for ADS1015 alert if connected to pin 3 of MCU (JP9)
     * @retval true if event occured
     */
    bool isAlert();

    /*!
     * @brief Check emergency stop if connected to pin 5 of MCU (JP10)
     * @retval true if emergeny stop pressed
     */
    bool isEmergencyStop();
    
  private:

    const uint16_t PCA_HIGH = 4095;
    const uint16_t PCA_LOW = 0;
    const uint16_t SERVO_MIN = 205;
    const uint16_t SERVO_MAX = 410;
    
    uint8_t pca9685Addr, ads1015Addr;

    uint32_t oscillatorFrequency;

    bool servo0Connected, servo1Connected, oeConnected, alertConnected, emsConnected;

    Servo *servo0;
    Servo *servo1;
    
    typedef struct PcaPinmap {
      
      uint8_t inh12 = 0;
      uint8_t inh34 = 1;
      uint8_t pwm1 = 2;
      uint8_t pwm2 = 3;
      uint8_t pwm3 = 4;
      uint8_t pwm4 = 5;

      uint8_t servo0 = 8;
      uint8_t servo1 = 9;

      uint8_t led3 = 10;
      uint8_t led2 = 11;
      uint8_t led1 = 12;
      uint8_t led0 = 13;

      uint8_t oe = 4;
      
    }PcaPinmap;

    PcaPinmap pcaPinmap;

    
   typedef struct AdsPinmap {
  
     uint8_t is12 = 0;
     uint8_t is34 = 1;
     uint8_t bemf12 = 2;
     uint8_t bemf34 = 3;
     
   }AdsPinmap;
    
   AdsPinmap adsPinmap;

   typedef struct McuPinmap {
  
     uint8_t oe = 4;
     uint8_t alert = 3;
     uint8_t ems = 5;

     uint8_t servo0 = A1;
     uint8_t servo1 = A3;
  
   }McuPinmap;

   McuPinmap mcuPinmap;
};

#endif
