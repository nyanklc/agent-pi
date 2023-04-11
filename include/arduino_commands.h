#ifndef __ARDUINO_COMMANDS_H_
#define __ARDUINO_COMMANDS_H_

struct ArduinoCommands {
    double linear_speed;          // m/s
    double angular_speed;         // rad/s
    double camera_angular_speed;  // rad/s

    void print(std::string msg = "") {
        if (msg != "")
            std::cout << msg << std::endl;
        std::cout << "linear_speed: " << linear_speed << ", angular_speed: " << angular_speed << ", camera_angular_speed: " << camera_angular_speed << "\n";
    }
};

#endif