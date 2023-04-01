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
#include "../include/controls.h"
#include "../include/globals.h"
#include "../include/gui_handler.h"
#include "../include/serial_handler.h"
#include "../include/stream_getter.h"
#include "../include/topdown.h"

bool initSystem(StreamGetter &stream_getter, Agent &agent,
                SerialHandler &serial_handler, GUIHandler &gui_handler, GUIHandler &gui_handler2) {
    if (!stream_getter.getRetrieved())
        return false;

    if (!stream_getter.startStream())
        return false;

    // wait until the stream thread starts
    while (!stream_getter.isReady())
        ;

    if (GUI_ON) {
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

    if (SERIAL_ON) {
        serial_handler.init(SERIAL_PORT, SERIAL_BAUDRATE);
        serial_handler.start();
        while (!serial_handler.isReady()) // unnecessary i think
            ;
    }

    return true;
}

int main(int argc, char **argv) {
    StreamGetter stream_getter(0);
    Agent agent;
    SerialHandler serial_handler;
    GUIHandler gui_handler;
    GUIHandler gui_handler_topdown;
    TopDown topdown;

    initSystem(stream_getter, agent, serial_handler, gui_handler, gui_handler_topdown);

    cv::Mat frame;
    cv::Mat frame_colored;
    bool first_run = true;
    while (1) {
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
        std::vector<TagPose> tag_objects = agent.process(frame);  // TODO: copying vector here, find a better way
        std::cout << "processing fps: " << cv::getTickFrequency() / (cv::getTickCount() - process_start_time) << "\n";

        // generate and send control commands
        ArduinoCommands arduino_commands = agent.getOutputCommands(tag_objects);
        // std::cout << "main sent: " << SerialHandler::constructFromCommands(arduino_commands) << std::endl;
        serial_handler.setCommand(arduino_commands);
        std::string received_msg = serial_handler.getMessage();
        // std::cout << "main received: " << received_msg << std::endl;
        // agent.setArduinoResponse(received_msg); // TODO: implement

        if (GUI_ON) {
            agent.drawDetections(frame_colored, DRAW_CUBES, DRAW_AXES);
            cv::resize(frame_colored, frame_colored, cv::Size(), 2, 2);
            gui_handler.setFrame(frame_colored);

            if (tag_objects.size() != 0) {
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
    serial_handler.stop();

    return 0;
}
