#pragma once


#include <KeplerBRAIN_V4.h>

// function to turn the motors in the right direction regardless of which direction the wires are connected
// max motor range is from -100 to 100
void m_motor_write(int port, double value)
{
    static int motor_multiplier_1 = -1;
    static int motor_multiplier_2 = -1;
    static int motor_multiplier_3 = 1;
    static int motor_multiplier_4 = 1;

    switch (port)
    {
    case 0:
        WRITE_MOTOR(M1, value * motor_multiplier_1);
        break;
    case 1:
        WRITE_MOTOR(M2, value * motor_multiplier_2);
        break;
    case 2:
        WRITE_MOTOR(M3, value * motor_multiplier_3);
        break;
    case 3:
        WRITE_MOTOR(M4, value * motor_multiplier_4);
        break;
    }
}

//function to collectively write to all motors at once
void multi_motor_write(double value1, double value2, double value3, double value4) {
    m_motor_write(0, value1);
    m_motor_write(1, value2);
    m_motor_write(2, value3);
    m_motor_write(3, value4);
}

// function to pull the distance data from the expansion board connected via spi
void read_tof_sensors(uint8_t *distance_values)
{

    // tof sensors on the roboter
    //   4 3
    // 5     2
    // 6     1
    //   7 0

    // read 8 Bytes from TOF Sensorboard BEGIN
    digitalWrite(SPI1, LOW);
    if (spi.transfer(0XFF) == 250)
    {
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

void read_imu(uint16_t *yaw, uint16_t *pitch, uint16_t *roll) {
    *yaw = READ_I2C_BNO055_YAW();
    *pitch = READ_I2C_BNO055_PITCH();
    *roll = READ_I2C_BNO055_ROLL();
}

// function to check if any button is pressed
bool any_button_is_pressed()
{
    if (READ_BUTTON_CLOSED(B1) == 1 || READ_BUTTON_CLOSED(B2) == 1 || READ_BUTTON_CLOSED(B3) == 1)
    {
        return true;
    }
    else
    {
        return false;
    }
}

// function to check if a button is pressed
int a_button_is_pressed()
{
    if (READ_BUTTON_CLOSED(B1) == 1)
    {
        return 1;
    }
    else if (READ_BUTTON_CLOSED(B2) == 1)
    {
        return 2;
    }
    else if (READ_BUTTON_CLOSED(B3) == 1)
    {
        return 3;
    }
    else
    {
        return 0;
    }
}

void write_lcd_text_clear(int x, int y, int choice, String text)
{
    if (choice < 1)
    {
        choice = 1;
    };
    if (choice > 3)
    {
        choice = 3;
    };

    switch (choice)
    {
    case 1:
        WRITE_LCD_TEXT(1, 1, "                ");
        WRITE_LCD_TEXT(x, y, text);
        break;
    case 2:
        WRITE_LCD_TEXT(1, 2, "                ");
        WRITE_LCD_TEXT(x, y, text);
        break;
    case 3:
        WRITE_LCD_CLEAR();
        WRITE_LCD_TEXT(x, y, text);
        break;
    }
}

// function to clear display line every 100th time it is called

void write_lcd_text_clear_multiple(int x, int y, int choice, String text)
{
    static uint mwltc_count;
    mwltc_count++;
    if (mwltc_count >= 120)
    {
        mwltc_count = 0;
        write_lcd_text_clear(x, y, choice, text);
    }
    else
    {
        WRITE_LCD_TEXT(x, y, text);
    }
}

// class where all helper, initialising etc. functions go
class Helper
{
public:
    void set_tof_offsets(int8_t *offset_values)
    {
        static const int distance_wall = 5;
        uint8_t distance_values[8];
        // calibrate left
        write_lcd_text_clear(1, 1, 3, "calibration left");
        do
        {
            read_tof_sensors(distance_values);
            write_lcd_text_clear_multiple(1, 2, 2, String(distance_values[5]) + " " + String(distance_values[6]));
        } while (READ_BUTTON_PRESSED(B1) != 1);
        offset_values[5] = (distance_values[5] - distance_wall);
        offset_values[6] = (distance_values[6] - distance_wall);
        // calibrate right
        write_lcd_text_clear(1, 1, 3, "calibration right");
        do
        {
            read_tof_sensors(distance_values);
            write_lcd_text_clear_multiple(1, 2, 2, String(distance_values[2]) + " " + String(distance_values[1]));
        } while (READ_BUTTON_PRESSED(B1) != 1);
        offset_values[2] = (distance_values[2] - distance_wall);
        offset_values[1] = (distance_values[1] - distance_wall);
        // calibrate front
        write_lcd_text_clear(1, 1, 3, "calibration front");
        do
        {
            read_tof_sensors(distance_values);
            write_lcd_text_clear_multiple(1, 2, 2, String(distance_values[4]) + " " + String(distance_values[3]));
        } while (READ_BUTTON_PRESSED(B1) != 1);
        offset_values[4] = (distance_values[4] - distance_wall);
        offset_values[3] = (distance_values[3] - distance_wall);
        // calibrate back
        write_lcd_text_clear(1, 1, 3, "calibration back");
        do
        {
            read_tof_sensors(distance_values);
            write_lcd_text_clear_multiple(1, 2, 2, String(distance_values[7]) + " " + String(distance_values[0]));
        } while (READ_BUTTON_PRESSED(B1) != 1);
        offset_values[7] = (distance_values[7] - distance_wall);
        offset_values[0] = (distance_values[0] - distance_wall);
    }
};

// test cases
class TestCases
{
public:
    void corner_turn(uint16_t degrees) {
        uint16_t yaw, pitch, roll;
        uint16_t oyaw, opitch, oroll;
        read_imu(&oyaw, &opitch, &oroll);
        SLEEP(500);
        read_imu(&oyaw, &opitch, &oroll);
        uint8_t distance_values[8];
        read_tof_sensors(distance_values);
        
        while (abs(oyaw-90) <= yaw) {
            read_imu(&yaw, &pitch, &roll);
            WRITE_LCD_TEXT(1,1, String(oyaw) + " " + String(abs(oyaw-90)));
            WRITE_LCD_TEXT(1,2, String(yaw));
            multi_motor_write(90, 90, -90, -90);
        }
        multi_motor_write(0,0,0,0);
    }

    void turn(uint degrees) {
        
    }

    void tof_display_raw()
    {
        static uint8_t distance_values[8];
        static uint8_t count = 0;
        read_tof_sensors(distance_values);
        WRITE_LCD_TEXT(1, 1, String(distance_values[5]) + " " + String(distance_values[4]) + " " + String(distance_values[3]) + " " + String(distance_values[2]));
        WRITE_LCD_TEXT(1, 2, String(distance_values[6]) + " " + String(distance_values[7]) + " " + String(distance_values[0]) + " " + String(distance_values[1]));
        if (count > 100)
        {
            count = 0;
            WRITE_LCD_CLEAR();
        }
        count++;
    }

    void motor_switch()
    {
        m_motor_write(0, 100);
        delay(1000);
        m_motor_write(0, 0);
        m_motor_write(1, 100);
        delay(1000);
        m_motor_write(1, 0);
        m_motor_write(2, 100);
        delay(1000);
        m_motor_write(2, 0);
        m_motor_write(3, 100);
        delay(1000);
        m_motor_write(3, 0);
    }

    void led_button_test()
    {
        if (READ_BUTTON_CLOSED(B1))
        {
            WRITE_LED(L1, 1);
        }
        else
        {
            WRITE_LED(L1, 0);
        }
        if (READ_BUTTON_CLOSED(B2))
        {
            WRITE_LED(L2, 1);
        }
        else
        {
            WRITE_LED(L2, 0);
        }
        if (READ_BUTTON_CLOSED(B3))
        {
            WRITE_LED(L3, 1);
        }
        else
        {
            WRITE_LED(L3, 0);
        }
    }

    void drive_parallel()
    // tof sensors on the roboter
    //   4 3
    // 5     2
    // 6     1
    //   7 0

    // drive parallel to a wall
    // sides: 0 left; 1 right
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
};
