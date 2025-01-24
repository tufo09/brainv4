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