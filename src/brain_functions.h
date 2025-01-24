#pragma once

#include <KeplerBRAIN_V4.h>


// multipliers, so the motors turn into the right direction
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