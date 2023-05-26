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
    dummy_commands.left_motor_speed = 0;
    dummy_commands.right_motor_speed = 0;
    dummy_commands.camera_step_count = 0;
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
    return std::to_string(commands.left_motor_speed) + " " + std::to_string(commands.right_motor_speed) + " " + std::to_string(commands.camera_step_count);
}

std::string SerialHandler::getMessage() {
    std::lock_guard<std::mutex> lock(mutex_receive_);
    return latest_received_;
}

void SerialHandler::communicationLoop() {
    while (running_) {
        if (message_to_send_.empty())
        {
            std::cout << "serial message to send is empty\n";
            continue;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // send
        {
            std::lock_guard<std::mutex> lock(mutex_send_);
            if (!message_to_send_.empty()) {
                serialPrintf(port_, "%s\n", message_to_send_.c_str());
                // std::cout << "serial sent: " << message_to_send_ << std::endl;
            }
        }
    }
}