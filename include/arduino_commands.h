#ifndef __ARDUINO_COMMANDS_H_
#define __ARDUINO_COMMANDS_H_

struct ArduinoCommands {
    double left_motor_speed;
    double right_motor_speed;
    double camera_step_count;

    void print(std::string msg = "") {
        if (msg != "")
            std::cout << msg << std::endl;
        std::cout << "left_motor_speed: " << left_motor_speed << ", right_motor_speed: " << right_motor_speed << ", camera_step_count: " << camera_step_count << "\n";
    }
};

#endif