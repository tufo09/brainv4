#include <Arduino.h>
#include "KeplerBRAIN_V4.h"
#include <brainV4.h>


uint8_t value_1;
uint8_t value_2;
uint8_t value_3;
uint8_t value_4;
uint8_t value_5;
uint8_t value_6;
uint8_t value_7;
uint8_t value_8;
uint8_t count;
 
void setup()
{
  KEPLERBRAIN_INIT();
  WRITE_LCD_CLEAR();
  WRITE_LCD_TEXT(1,1,"it dklsa");
  WRITE_I2C_BNO055_INIT();
}
 
void loop()
{
}


int debug() {

}

void system() {
  // read 8 Bytes from TOF Sensorboard BEGIN
 
  digitalWrite(SPI1, LOW);
 
  if(spi.transfer(0XFF) == 250)
  { 
    value_1 = spi.transfer(0XFF);
    value_2 = spi.transfer(0XFF);
    value_3 = spi.transfer(0XFF);
    value_4 = spi.transfer(0XFF);
    value_5 = spi.transfer(0XFF);
    value_6 = spi.transfer(0XFF);
    value_7 = spi.transfer(0XFF);
    value_8 = spi.transfer(0XFF);
  }
 
  digitalWrite(SPI1, HIGH);
 
  // read 8 Bytes from TOF Sensorboard END
 
  WRITE_LCD_TEXT(1, 1, String(value_6)+" "+String(value_5)+" "+String(value_4)+" "+String(value_3));
  WRITE_LCD_TEXT(1, 2, String(value_7)+" "+String(value_8)+" "+String(value_1)+" "+String(value_2));
  if (count>100) {
    count = 0;
    WRITE_LCD_CLEAR();
  }
  // val6   val5    val4   val3
  // val7   val8    val1    val2
  count ++;
}
