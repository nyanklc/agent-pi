#include <chrono>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <type_traits>

#include "../include/agent.h"
#include "../include/arduino_commands.h"
#include "../include/globals.h"
#include "../include/gui_handler.h"
#include "../include/stream_getter.h"
#include "../include/topdown.h"

// RPi I/O library
#include <wiringSerial.h>

int SERIAL = 0;

void initSerial(std::string port, int baudrate)
{
    SERIAL = serialOpen(port.c_str(), baudrate);
    if (SERIAL < 0)
    {
        std::cerr << "Error opening serial connection.\n";
        exit(1);
    }
}

bool initSystem(StreamGetter &stream_getter, Agent &agent, GUIHandler &gui_handler, GUIHandler &gui_handler2)
{
    if (!stream_getter.getRetrieved())
        return false;

    if (!stream_getter.startStream())
        return false;

    // wait until the stream thread starts
    while (!stream_getter.isReady())
        ;

    if (SERIAL_ON)
    {
        initSerial(SERIAL_PORT, SERIAL_BAUDRATE);
    }

    if (GUI_ON)
    {
        if (!gui_handler.start("agent-pi"))
            return false;

        while (!gui_handler.isReady())
            ;

        // topdown
        if (!gui_handler2.start("topdown view"))
            return false;

        while (!gui_handler2.isReady())
            ;
    }

    return true;
}

std::string constructFromCommands(const ArduinoCommands &commands)
{
    return std::to_string(commands.left_motor_speed) + " " + std::to_string(commands.right_motor_speed) + " " + std::to_string(commands.camera_step_count) + '\n';
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
    try
    {
        std::cout << serialDataAvail(SERIAL) << std::endl;
        if (serialDataAvail(SERIAL) == 0)
        {
            // std::cout << "haven't received any messages yet\n";
            return false;
        }
        if (serialDataAvail(SERIAL) == -1)
        {
            std::cout << "serial receive error" << std::endl;
            throw;
        }

        int ch = serialGetchar(SERIAL);
        if ((char)ch == 'a')
        {
            std::cout << "OK received.\n";
            return true;
        }
        else
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

int main(int argc, char **argv)
{
    std::cout << "### INITIALIZING ###\n";
    StreamGetter stream_getter(0);
    Agent agent;
    GUIHandler gui_handler;
    GUIHandler gui_handler_topdown;
    TopDown topdown;

    initSystem(stream_getter, agent, gui_handler, gui_handler_topdown);

    cv::Mat frame;
    cv::Mat frame_colored;
    bool first_run = true;
    std::cout << "### LET'S GOOOOOOOOOOOOOOOOOOOOOOOOOOOOO ###\n";
    while (1)
    {
        // auto stream_start = cv::getTickCount();
        // std::cout << "start: " << stream_start << "\n";

        // get frame
        // do not process the same frame again
        if (!stream_getter.isUpdated())
            continue;

        // quit if stream wasn't read
        if (!stream_getter.getRetrieved())
            break;

        // auto stream_ready = cv::getTickCount();
        // std::cout << "ready: " << stream_start << "\n";

        frame = stream_getter.getFrameGray();
        if (GUI_ON)
            frame_colored = stream_getter.getFrame();

        if (frame.empty())
            break;
        // std::cout << "start @ " << (cv::getTickCount() - stream_start)/ cv::getTickFrequency() << " fps\n";
        // std::cout << "ready @ " << (cv::getTickCount() - stream_ready)/ cv::getTickFrequency() << "fps\n";

        // process
        auto process_start_time = cv::getTickCount();
        std::vector<TagPose> tag_objects = agent.process(frame); // TODO: copying vector here, find a better way
        // std::cout << "processing fps: " << cv::getTickFrequency() / (cv::getTickCount() - process_start_time) << "\n";

        if (tag_objects.size() > 0)
        {
            std::cout << "detection\n";
            // generate and send control commands
            ArduinoCommands arduino_commands = agent.getOutputCommands(tag_objects);

            if (!first_run)
            {
                while(!receiveMessage());
            }
            sendMessage(arduino_commands);

            first_run = false;
        }
        else
        {
            std::cout << "no detection\n";
            ArduinoCommands arduino_commands;
            arduino_commands.left_motor_speed = 0;
            arduino_commands.right_motor_speed = 0;
            arduino_commands.camera_step_count = 0;

            if (!first_run)
            {
                while(!receiveMessage());
            }
            sendMessage(arduino_commands);

            first_run = false;
        }

        if (GUI_ON)
        {
            agent.drawDetections(frame_colored, DRAW_CUBES, DRAW_AXES);
            cv::resize(frame_colored, frame_colored, cv::Size(), 2, 2);
            gui_handler.setFrame(frame_colored);

            if (tag_objects.size() != 0)
            {
                std::vector<TopDownObject> topdown_objects = TopDown::convertToTopDown(tag_objects);
                auto window_size = frame_colored.size();
                cv::Mat f = topdown.prepareView(topdown_objects, window_size);
                gui_handler_topdown.setFrame(f);
            }
        }
    }

    // yes
    stream_getter.stopStream();
    gui_handler.stop();
    gui_handler_topdown.stop();

    return 0;
}
