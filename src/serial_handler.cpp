#include "../include/serial_handler.h"

void SerialHandler::init(std::string port, int baudrate) {
    port_name_ = port;
    baudrate_ = baudrate;
    port_ = serialOpen(port_name_.c_str(), baudrate_);
    if (port_ < 0) {
        std::cerr << "Error opening serial connection.\n";
        exit(1);
    }

    ArduinoCommands dummy_commands;
    dummy_commands.linear_speed = 0;
    dummy_commands.angular_speed = 0;
    dummy_commands.camera_angular_speed = 0;
    command_to_send_ = dummy_commands;

    running_ = false;
}

void SerialHandler::start() {
    running_ = true;
    th_ = std::thread(&SerialHandler::communicationLoop, this);
}

void SerialHandler::stop() {
    th_.join();
    serialClose(port_);
    running_ = false;
}

bool SerialHandler::isReady() {
    return running_;
}

void SerialHandler::setCommand(const ArduinoCommands &commands) {
    std::lock_guard<std::mutex> lock(mutex_send_);
    message_to_send_ = constructFromCommands(commands);
}

std::string SerialHandler::constructFromCommands(const ArduinoCommands &commands) {
    return std::to_string(commands.linear_speed) + " " + std::to_string(commands.angular_speed) + " " + std::to_string(commands.camera_angular_speed);
}

std::string SerialHandler::getMessage() {
    std::lock_guard<std::mutex> lock(mutex_receive_);
    return latest_received_;
}

void SerialHandler::communicationLoop() {
    while (running_) {
        // send
        {
            std::lock_guard<std::mutex> lock(mutex_send_);
            if (!message_to_send_.empty()) {
                serialPrintf(port_, "%s\n", message_to_send_.c_str());
                // std::cout << sizeof(message_to_send_.c_str()) / sizeof(char) << "\n";
                // std::cout << "serial sent: " << message_to_send_ << std::endl;
            }
        }
        serialFlush(port_); // waits for the transmission of outgoing serial data to complete

        // wait until arduino responds back
        while (serialDataAvail(port_) < SERIAL_RECEIVE_NUM_OF_BYTES);

        // receive
        std::string current_received;
        while (serialDataAvail(port_) > 0) {
            char c = serialGetchar(port_);
            if (c == '\n') {
                std::lock_guard<std::mutex> lock(mutex_receive_);
                latest_received_ = current_received;
                // std::cout << "serial received: " << current_received << std::endl;
                // break; // ??
            } else {
                current_received += c;
            }
        }
    }
}