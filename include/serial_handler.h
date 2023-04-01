#ifndef __SERIAL_HANDLER_H
#define __SERIAL_HANDLER_H

// RPi I/O library
#include <wiringSerial.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include "arduino_commands.h"

class SerialHandler {
   public:
    SerialHandler() {}  // dummy

    void init(std::string port, int baudrate);

    void start();

    void stop();

    bool isReady();

    void setCommand(const ArduinoCommands &commands);

    std::string getMessage();

    static std::string constructFromCommands(const ArduinoCommands &commands);

   private:
    void communicationLoop();

    std::string port_name_;
    int port_;
    int baudrate_;
    bool running_;
    ArduinoCommands command_to_send_;
    std::string message_to_send_;
    std::string latest_received_;

    std::thread th_;
    std::mutex mutex_send_;
    std::mutex mutex_receive_;
};

#endif
