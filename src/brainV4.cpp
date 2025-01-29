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
  drive_parallel();
}

// tof sensors on the roboter
//   4 3
// 5     2
// 6     1
//   7 0

// drive parallel to a wall
// sides: 0 left; 1 right
void drive_parallel()
{
  uint8_t tof_read_arr[8];
  int8_t ground_speed = 100;
  int8_t speed_left, speed_right;
  uint8_t target_distance = 15; // Target distance from the wall in centimeters

  while (a_button_is_pressed() == 0)
  {
    read_tof_sensors(tof_read_arr);

    uint8_t front_left_distance = tof_read_arr[5];
    uint8_t back_left_distance = tof_read_arr[6];

    // Adjust speed based on distance from the wall
    if (front_left_distance > target_distance)
    {
      speed_right = 100;
      speed_left = 50;
    }
    else if (front_left_distance < target_distance)
    {
      speed_left = 100;
      speed_right = 50;
    }
    else
    {
      speed_left = 70;
      speed_right = 70;
    }

    // Write motor speeds
    m_motor_write(0, speed_right); // Left motors
    m_motor_write(1, speed_right); // Right motors
    m_motor_write(2, speed_left);  // Left motors
    m_motor_write(3, speed_left);  // Right motors
  }
}
