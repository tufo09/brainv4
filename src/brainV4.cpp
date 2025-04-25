#include <Arduino.h>
#include <KeplerBRAIN_V4.h>
#include <brainV4.h>
#include <brain_functions.h>

TestCases testCases;
Helper helper;

void setup()
{
  KEPLERBRAIN_INIT();
  WRITE_I2C_BNO055_INIT();
  WRITE_LCD_CLEAR();
  
}

void loop()
{
  testCases.corner_turn(90);
  
}