#include <chrono>
#include <iostream>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <type_traits>

#include "../include/agent.h"
#include "../include/globals.h"
#include "../include/gui_handler.h"
#include "../include/stream_getter.h"
#include "../include/topdown.h"
#include "../include/arduino_commands.h"
#include "../include/external/PID.h"
#include "../include/simple_controller.h"
#include <chrono>
#include <thread>

// RPi I/O library
#include <wiringSerial.h>

// Agent & Controllers
Agent agent;
ArduinoCommands last_command;
bool commands_updated = false;

int getLeftSpeed()
{
    return last_command.left_motor_speed;
}

int getRightSpeed()
{
    return last_command.right_motor_speed;
}

void setLeftSpeed(int output)
{
    if (output != last_command.left_motor_speed)
        commands_updated = true;
    last_command.left_motor_speed = output;
}

void setRightSpeed(int output)
{
    if (output != last_command.right_motor_speed)
        commands_updated = true;
    last_command.right_motor_speed = output;
}

auto left_controller_ = std::make_unique<PIDController<int>>(LEFT_MOTOR_P, LEFT_MOTOR_I, LEFT_MOTOR_D, getLeftSpeed, setLeftSpeed);
auto right_controller_ = std::make_unique<PIDController<int>>(RIGHT_MOTOR_P, RIGHT_MOTOR_I, RIGHT_MOTOR_D, getRightSpeed, setRightSpeed);

SimpleController left_controller_simple_;
SimpleController right_controller_simple_;

int SERIAL = 0;
bool first_run = true;

std::string last_message = "";

bool sent_message = false;

void initSerial(std::string port, int baudrate)
{
    SERIAL = serialOpen(port.c_str(), baudrate);
    if (SERIAL < 0)
    {
        std::cerr << "Error opening serial connection.\n";
        exit(1);
    }
}

std::string constructFromCommands(const ArduinoCommands &commands)
{
    return std::to_string(commands.left_motor_speed) + " " + std::to_string(commands.right_motor_speed) + " " + std::to_string(commands.camera_step_count) + '\n';
}

void sendMessage(ArduinoCommands &commands)
{
    // if (commands.left_motor_speed < MIN_MOTOR_ANALOG && commands.left_motor_speed > -MIN_MOTOR_ANALOG)
    //     commands.left_motor_speed = 0;
    // if (commands.right_motor_speed < MIN_MOTOR_ANALOG && commands.right_motor_speed > -MIN_MOTOR_ANALOG)
    //     commands.right_motor_speed = 0;
    std::string message = constructFromCommands(commands);

    serialPrintf(SERIAL, "%s\n", message.c_str());
    std::cout << "serial sent: " << message << std::endl;
}

bool receiveMessage()
{
    // receive
    try
    {
        // std::cout << serialDataAvail(SERIAL) << std::endl;
        if (serialDataAvail(SERIAL) == 0)
        {
            std::cout << "haven't received any messages yet\n";
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
            // std::cout << "OK received.\n";
            while (serialDataAvail(SERIAL)) char _ = serialGetchar(SERIAL);
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

inline ArduinoCommands getOutput(std::vector<TagPose> &tag_objects, Agent &agent)
{
    ArduinoCommands ret;
    ret = agent.getOutputCommands(tag_objects);
    return ret;
}

void sendOutput(ArduinoCommands &arduino_commands)
{
    if (!SERIAL_ON)
        return;

    sendMessage(arduino_commands);
    first_run = false;

    using namespace std::chrono_literals;
    while(!receiveMessage()) std::this_thread::sleep_for(50ms);
}

void showOnGUI(
    Agent &agent,
    GUIHandler &gui_handler,
    cv::Mat &frame_colored,
    TopDown &topdown,
    GUIHandler &gui_handler_topdown,
    std::vector<TagPose> &tag_objects)
{
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

long unsigned int getTime()
{
    // auto currentTime = std::chrono::system_clock::now();
    // auto timeInSeconds = std::chrono::time_point_cast<std::chrono::seconds>(currentTime);
    // int timeAsInt = timeInSeconds.time_since_epoch().count();
    // std::cout << "Current time as an integer: " << timeAsInt << std::endl;

    auto x = cv::getTickCount() / cv::getTickFrequency();
    long unsigned int int_x = (int)x * 100;

    return int_x;
}

bool initSystem(Agent &agent, GUIHandler &gui_handler, GUIHandler &gui_handler2)
{
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

    // Arduino resets after receiving first message??
    std::cout << "resetting Arduino\n";
    ArduinoCommands init_command;
    init_command.left_motor_speed = 0;
    init_command.right_motor_speed = 0;
    init_command.camera_step_count = 0;
    sendMessage(init_command);
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(5000ms);

    sent_message = false;
    last_command.left_motor_speed = 0;
    last_command.right_motor_speed = 0;
    last_command.camera_step_count = 0;

    left_controller_->registerTimeFunction(getTime);
    right_controller_->registerTimeFunction(getTime);

    left_controller_->setFeedbackWrapped(false);
    right_controller_->setFeedbackWrapped(false);

    left_controller_simple_.init(LEFT_STEP_AMOUNT, LEFT_TOLERANCE, MIN_MOTOR_ANALOG);
    right_controller_simple_.init(RIGHT_STEP_AMOUNT, RIGHT_TOLERANCE, MIN_MOTOR_ANALOG);

    return true;
}

void readFrame(cv::VideoCapture &cvCap, cv::Mat &frame, cv::Mat &frame_colored, cv::Mat &last_frame)
{
    try
    {
        if(!cvCap.read(frame_colored)) throw;

        cv::cvtColor(frame_colored, frame, cv::COLOR_BGRA2GRAY);
        cv::resize(frame, frame, cv::Size(), 0.5, 0.5);

        if (!first_run)
        {
            cv::Mat diff = frame - last_frame;
            while (!cv::countNonZero(diff))
            {
                if(!cvCap.read(frame)) throw;
                diff = frame - last_frame;
            }
        }
        last_frame = frame;


        if (GUI_ON)
                cv::resize(frame_colored, frame_colored, cv::Size(), 0.5, 0.5);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}

void tickControllers(ArduinoCommands &output)
{
    left_controller_->setTarget(output.left_motor_speed);
    left_controller_->tick();
    right_controller_->setTarget(output.right_motor_speed);
    right_controller_->tick();
}

void tickSimpleControllers(ArduinoCommands &output)
{
    left_controller_simple_.setGoal(output.left_motor_speed);
    last_command.left_motor_speed += left_controller_simple_.update(last_command.left_motor_speed);
    right_controller_simple_.setGoal(output.right_motor_speed);
    last_command.right_motor_speed += right_controller_simple_.update(last_command.right_motor_speed);
}

bool areArduinoCommandsSame(ArduinoCommands &output, ArduinoCommands &last_command)
{
    if (output.camera_step_count == last_command.camera_step_count)
        if (output.left_motor_speed == last_command.left_motor_speed)
            if (output.right_motor_speed == last_command.right_motor_speed)
                return true;
    return false;
}

bool shouldCameraTurn(ArduinoCommands &output)
{
    return output.camera_step_count != 0;
}

int main(int argc, char **argv)
{
    std::cout << "### INITIALIZING ###\n";
    GUIHandler gui_handler;
    GUIHandler gui_handler_topdown;
    TopDown topdown;

    cv::VideoCapture cvCap = cv::VideoCapture(VIDEO_SOURCE);

    initSystem(agent, gui_handler, gui_handler_topdown);

    cv::Mat frame;
    cv::Mat frame_colored;
    cv::Mat last_frame;
    std::cout << "### LET'S GOOOOOOOOOOOOOOOOOOOOOOOOOOOOO ###\n";

    // ArduinoCommands com;
    // com.left_motor_speed = 255;
    // com.right_motor_speed = 255;
    // com.camera_step_count = 0;
    // sendMessage(com);

    while (1)
    {
        std::cout << "time: " << getTime() << std::endl;

        auto loop_start_time = cv::getTickCount();
        // std::cout << "start: " << loop_start_time << "\n";

        readFrame(cvCap, frame, frame_colored, last_frame);

        // int error = 0;
        // if (!getFrame(stream_getter, frame, frame_colored, error))
        //     continue;
        // if (error)
        //     break;

        // process
        auto process_start_time = cv::getTickCount();
        std::vector<TagPose> tag_objects = agent.process(frame); // TODO: copying vector here, find a better way
        // std::cout << "processing fps: " << cv::getTickFrequency() / (cv::getTickCount() - process_start_time) << "\n";

        // output
        ArduinoCommands output = getOutput(tag_objects, agent);
        output.print("COMMANDS OUTPUT");
        if (!areArduinoCommandsSame(output, last_command))
        {
            last_command.camera_step_count = output.camera_step_count;

            /* TO CHANGE CONTROLLER, UNCOMMENT DESIRED ONE */

            /* PID External */
            // std::cout << "HEYHEY BEFORE: " << std::endl;
            // last_command.print("BEFORE");
            // tickControllers(output);
            // std::cout << "HEYHEY AFTER: " << std::endl;
            // last_command.print("AFTER");

            /* Simple Controller */
            std::cout << "HEYHEY BEFORE: " << std::endl;
            last_command.print("BEFORE");
            tickSimpleControllers(output);
            std::cout << "HEYHEY AFTER: " << std::endl;
            last_command.print("BEFORE");

            /* no controller */
            // last_command.left_motor_speed = output.left_motor_speed;
            // last_command.right_motor_speed = output.right_motor_speed;

            sendOutput(last_command);
        }
        else
        {
            if (shouldCameraTurn(output))
            {
                last_command.camera_step_count = output.camera_step_count;
                sendOutput(last_command);
            }
        }

        // debug
        showOnGUI(agent, gui_handler, frame_colored, topdown, gui_handler_topdown, tag_objects);

        std::cout << "loop fps: " << cv::getTickFrequency() / (cv::getTickCount() - loop_start_time) << "\n";
    }

    // yes
    gui_handler.stop();
    gui_handler_topdown.stop();

    return 0;
}
