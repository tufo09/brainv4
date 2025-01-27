#include <Arduino.h>
#include <KeplerBRAIN_V4.h>
#include <brainV4.h>
#include <brain_functions.h>

TestCases testCases;
Helper helper;

int8_t offsets[8];
uint8_t tof_read_arr[8];

void setup()
{
  KEPLERBRAIN_INIT();
  WRITE_I2C_BNO055_INIT();
  WRITE_LCD_CLEAR();
}

void loop()
{
}

// tof sensors on the roboter
//   4 3
// 5     2
// 6     1
//   7 0

// drive parallel to a wall
// sides: 0 left; 1 right
void drive_parallel(int side)
{
  read_tof_sensors(tof_read_arr);
  // TODO: Modify the code for the sensor board to send in higher resolution. Right now it only sends in i think 1cm values, so one increment in a value is about a one centimetre increase in distance from the item it is measuring from. Code can be found at https://www.lbotics.at/keplerbrain-v4/sensoren/tof-abstandssensor.
  switch (side)
  {
  case 0:
    // left side
    // tof sensors to use are indexed with 5 and 6 in the array where five is the more forward sensor
    break;

  case 1:
    // right side
    // tof sensors to use are indexed with 2 and 1 in the array where two is the more foreward sensor
    break;
  }
}