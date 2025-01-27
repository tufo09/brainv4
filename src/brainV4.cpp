#include <Arduino.h>
#include <KeplerBRAIN_V4.h>
#include <brainV4.h>
#include <brain_functions.h>

TestCases testCases;
Maze maze;
Helper helper;

int8_t offsets[8];


TestCases testCases;
Maze maze;
Helper helper;

int8_t offsets[8];


void setup()
{
  KEPLERBRAIN_INIT();
  WRITE_I2C_BNO055_INIT();
  WRITE_LCD_CLEAR();

  helper.set_tof_offsets(offsets);
}


void loop() {
  WRITE_LED(L2,1);
  do { testCases.tof_display_raw();} while (READ_BUTTON_PRESSED(B2) != 1);
  WRITE_LED(L2,0);
  WRITE_LED(L3,1);
  do { testCases.tof_display_processed(offsets);} while (READ_BUTTON_PRESSED(B2) != 1);
  WRITE_LED(L3,0);
}