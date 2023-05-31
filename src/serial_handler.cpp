#include "../include/serial_handler.h"

void SerialHandler::init(std::string port, int baudrate)
{
    port_name_ = port;
    baudrate_ = baudrate;
    port_ = serialOpen(port_name_.c_str(), baudrate_);
    if (port_ < 0)
    {
        std::cerr << "Error opening serial connection.\n";
        exit(1);
    }

    ArduinoCommands dummy_commands;
    dummy_commands.left_motor_speed = 0;
    dummy_commands.right_motor_speed = 0;
    dummy_commands.camera_step_count = 0;
    command_to_send_ = dummy_commands;

    running_ = false;
    main_read_ = false;
    new_send_ = false;
    new_receive_ = false;
}

void SerialHandler::start()
{
    running_ = true;
    th_ = std::thread(&SerialHandler::communicationLoop, this);
}

void SerialHandler::stop()
{
    th_.join();
    serialClose(port_);
    running_ = false;
}

bool SerialHandler::isReady()
{
    return running_;
}

void SerialHandler::setCommand(const ArduinoCommands &commands)
{
    std::lock_guard<std::mutex> lock(mutex_send_);
    message_to_send_ = constructFromCommands(commands);
    std::cout << "set command\n";
    new_send_ = true;
}

std::string SerialHandler::constructFromCommands(const ArduinoCommands &commands)
{
    return std::to_string(commands.left_motor_speed) + " " + std::to_string(commands.right_motor_speed) + " " + std::to_string(commands.camera_step_count) + '\n';
}

bool SerialHandler::getMessage()
{
    std::lock_guard<std::mutex> lock(mutex_receive_);
    if (!new_receive_)
        return false;
    else
    {
        main_read_ = true;
        new_receive_ = false;
        std::cout << "main received response\n";
        return true;
    }
}

bool SerialHandler::sendMessage()
{

    if (message_to_send_.empty())
    {
        std::cout << "serial message to send is empty\n";
        return false;
    }

    if (new_send_)
    {
        std::lock_guard<std::mutex> lock(mutex_send_);
        new_send_ = false;
        serialPrintf(port_, "%s\n", message_to_send_.c_str());
        std::cout << "serial sent: " << message_to_send_ << std::endl;
        return true;
    }

    return false;
}

bool SerialHandler::receiveMessage()
{
    // receive
    try
    {
        std::cout << "serial receiving\n";
        if (serialDataAvail(port_) == 0)
        {
            std::cout << "haven't received any messages yet\n";
            return false;
        }
        if (serialDataAvail(port_) == -1)
        {
            std::cout << "serial receive error" << std::endl;
            throw;
        }

        int ch = serialGetchar(port_);
        std::cout << "serialgetchar: " << ch << std::endl;
        if ((char)ch == 'a')
        {
            std::lock_guard<std::mutex> lock(mutex_receive_);
            std::cout << "OK received.\n";
            new_receive_ = true;
            return true;
        }
        else if ((char)ch == 'b')
        {
            std::cerr << "something went wrong in response message from Arduino\n";
            throw;
        }
    }
    catch (const std::exception &e)
    {
        std::cout << "serial receive error\n";
        std::cerr << e.what() << '\n';
    }

    return false;
}

bool SerialHandler::checkMainReceived()
{
    // check if main received response
    std::cout << "check if main received response\n";
    std::lock_guard<std::mutex> lock(mutex_receive_);

    if (main_read_)
    {
        main_read_ = false;
        return true;
    }
    else
        return false;
}

void SerialHandler::communicationLoop()
{
    while (running_)
    {
        while (!sendMessage()) std::this_thread::sleep_for(std::chrono::milliseconds(50));

        while (!receiveMessage());

        if (!checkMainReceived())
            continue;
    }
}