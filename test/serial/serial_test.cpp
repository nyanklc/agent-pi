#include "../../include/serial_handler.h"
#include "../../include/arduino_commands.h"

int main(int argc, char **argv) {

    SerialHandler serial_handler;
    serial_handler.init("/dev/ttyACM0", 9600);
    serial_handler.start();
    while (!serial_handler.isReady());

    std::string msg_past;
    while (true) {
        ArduinoCommands commands;
        commands.linear_speed = 1.11;
        commands.angular_speed = -2.22;
        commands.camera_angular_speed = 3.33;
        serial_handler.setCommand(commands);
        // std::cout << "sent: " << SerialHandler::constructFromCommands(commands) << "\n";

        auto msg = serial_handler.getMessage();
        if (msg != msg_past) {
            uint8_t received_count = msg[3];
            std::cout << "received: " << (int)received_count << "\n";
        }

        msg_past = msg;
    }

    serial_handler.stop();

    return 0;
}