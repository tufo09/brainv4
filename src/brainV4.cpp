#include <Arduino.h>
#include <KeplerBRAIN_V4.h>
#include <brainV4.h>
#include <brain_functions.h>

void setup() {
  KEPLERBRAIN_INIT();
  WRITE_LCD_CLEAR();
  WRITE_I2C_BNO055_INIT();
}
 
void loop() {
  TestCases testCases;
  testCases.tof_display_raw();
}