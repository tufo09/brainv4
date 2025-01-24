#include <Arduino.h>
#include <KeplerBRAIN_V4.h>
#include <brainV4.h>
#include <brain_functions.h>

uint8_t distance_values[8];
uint8_t count = 0;

 
void setup()
{
  KEPLERBRAIN_INIT();
  WRITE_LCD_CLEAR();
  WRITE_I2C_BNO055_INIT();
}
 
void loop()
{
  m_motor_write(0,100);
  delay(1000);
  m_motor_write(0,0);
  m_motor_write(1,100);
  delay(1000);
  m_motor_write(1,0);
  m_motor_write(2,100);
  delay(1000);
  m_motor_write(2,0);
  m_motor_write(3,100);
  delay(1000);
  m_motor_write(3,0);


  read_tof_sensors(distance_values);
  WRITE_LCD_TEXT(1, 1, String(distance_values[5])+" "+String(distance_values[4])+" "+String(distance_values[3])+" "+String(distance_values[2]));
  WRITE_LCD_TEXT(1, 2, String(distance_values[6])+" "+String(distance_values[7])+" "+String(distance_values[0])+" "+String(distance_values[1]));
  if (count>100) {
    count = 0;
    WRITE_LCD_CLEAR();
  }
  // val6   val5    val4   val3
  // val7   val8    val1    val2
  count ++;

}