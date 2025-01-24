#include <Arduino.h>
#include <KeplerBRAIN_V4.h>
#include <brainV4.h>
#include <brain_functions.h>

void setup() {
  KEPLERBRAIN_INIT();
  WRITE_LCD_CLEAR();
  WRITE_I2C_BNO055_INIT();
}
 

TestCases testCases;
void loop() {
  testCases.tof_display_raw();
}