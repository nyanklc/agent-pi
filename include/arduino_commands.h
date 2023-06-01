#ifndef __ARDUINO_COMMANDS_H_
#define __ARDUINO_COMMANDS_H_

#include <string>
#include <iostream>

struct ArduinoCommands
{
    int left_motor_speed;
    int right_motor_speed;
    int camera_step_count;

    void print(std::string msg = "")
    {
        if (msg != "")
            std::cout << msg << std::endl;
        std::cout << "left_motor_speed: " << left_motor_speed << ", right_motor_speed: " << right_motor_speed << ", camera_step_count: " << camera_step_count << "\n";
    }
};

#endif