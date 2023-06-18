#include "../../include/arduino_commands.h"
#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <iomanip>
#include <sstream>

// RPi I/O library
#include <wiringSerial.h>

int SERIAL = 0;

ArduinoCommands hello()
{
    ArduinoCommands com;
    com.camera_step_count = 0;
    com.left_motor_speed = 0;
    com.right_motor_speed = 0;
    return com;
}

std::string constructFromCommands(const ArduinoCommands &commands)
{
    std::string msg = "";

    std::stringstream stream;
    stream << std::fixed << std::setprecision(2) << commands.left_motor_speed;
    std::string s = stream.str();
    msg += s;
    msg += " ";

    std::stringstream stream1;
    stream1 << std::fixed << std::setprecision(2) << commands.right_motor_speed;
    std::string s1 = stream1.str();
    msg += s1;
    msg += " ";

    std::stringstream stream2;
    stream2 << std::fixed << std::setprecision(2) << commands.right_motor_speed;
    std::string s2 = stream2.str();
    msg += s2;
    msg += "\n";

    return msg;
}

void initSerial(std::string port, int baudrate)
{
    SERIAL = serialOpen(port.c_str(), baudrate);
    if (SERIAL < 0)
    {
        std::cerr << "Error opening serial connection.\n";
        exit(1);
    }
}

void sendMessage(ArduinoCommands &commands)
{
    std::string message = constructFromCommands(commands);
    serialPrintf(SERIAL, "%s\n", message.c_str());
    std::cout << "serial sent: " << message << std::endl;
}

bool receiveMessage()
{
    // receive
    int ch = serialGetchar(SERIAL);
    if ((char)ch == 'a')
    {
        std::cout << "OK received.\n";
        return true;
    }
    else
    {
        std::cerr << "something went wrong in response message from Arduino\n";
        return false;
    }
}

int main(int argc, char **argv)
{
    using namespace std::chrono_literals;

    initSerial("/dev/ttyACM0", 9600);

    ArduinoCommands init_command = hello();
    sendMessage(init_command);
    std::this_thread::sleep_for(5000ms);

    bool first_run = true;
    while (1)
    {
        ArduinoCommands command = hello();

        if (!first_run)
        {
            while(!receiveMessage()) std::this_thread::sleep_for(2000ms);
        }
        sendMessage(command);

        first_run = false;
    }

    return 0;
}