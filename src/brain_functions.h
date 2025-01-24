#pragma once

#include <KeplerBRAIN_V4.h>


// function to turn the motors in the right direction regardless of which direction the wires are connected
int motor_multiplier_1 = -1;
int motor_multiplier_2 = -1;
int motor_multiplier_3 = 1;
int motor_multiplier_4 = 1;

void m_motor_write(int port, double value) {
    switch (port)
    {
    case 0:
        WRITE_MOTOR(M1, value*motor_multiplier_1);
        break;
    case 1:
        WRITE_MOTOR(M2, value*motor_multiplier_2);
        break;
    case 2:
        WRITE_MOTOR(M3, value*motor_multiplier_3);
        break;
    case 3:
        WRITE_MOTOR(M4, value*motor_multiplier_4);
        break;
    }
}


// function to pull the distance data from the expansion board connected via spi
void read_tof_sensors(uint8_t *distance_values) {

  // read 8 Bytes from TOF Sensorboard BEGIN
  digitalWrite(SPI1, LOW);
  if(spi.transfer(0XFF) == 250) { 
    distance_values[0] = spi.transfer(0XFF);
    distance_values[1] = spi.transfer(0XFF);
    distance_values[2] = spi.transfer(0XFF);
    distance_values[3] = spi.transfer(0XFF);
    distance_values[4] = spi.transfer(0XFF);
    distance_values[5] = spi.transfer(0XFF);
    distance_values[6] = spi.transfer(0XFF);
    distance_values[7] = spi.transfer(0XFF);
  }
  digitalWrite(SPI1, HIGH);
}
// function to check if any button is pressed
bool abutton_is_pressed() {
    if (READ_BUTTON_CLOSED(B1)==1||READ_BUTTON_CLOSED(B2)==1||READ_BUTTON_CLOSED(B3)==1) {return true;} else {return false;}
}
// test cases

// tof_display_raw
uint8_t distance_values[8];
uint8_t count = 0;

class TestCases {
    public:
        void tof_display_raw() {
            static uint8_t distance_values[8];
            static uint8_t count = 0;
            read_tof_sensors(distance_values);
            WRITE_LCD_TEXT(1, 1, String(distance_values[5])+" "+String(distance_values[4])+" "+String(distance_values[3])+" "+String(distance_values[2]));
            WRITE_LCD_TEXT(1, 2, String(distance_values[6])+" "+String(distance_values[7])+" "+String(distance_values[0])+" "+String(distance_values[1]));
            if (count>100) {
                count = 0;
                WRITE_LCD_CLEAR();
            }
            count ++;
        }

        void motor_switch() {
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
        }
};