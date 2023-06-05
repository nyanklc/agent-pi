#ifndef __AGENT_H
#define __AGENT_H

#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <bits/stdc++.h>

#include "april_tag_detector.h"
#include "arduino_commands.h"
#include "globals.h"
#include "camera_controller.h"
#include "simple_controller.h"
#include "pid.h"

struct GoalPose
{
    double x;
    double y;
    double yaw;

    void print(std::string name)
    {
        std::cout << name << " x: " << x << " y: " << y << " yaw: " << yaw << "\n";
    }
};

class Agent
{
public:
    Agent();

    std::vector<TagPose> process(cv::Mat &frame);

    void drawDetections(cv::Mat &frame, bool cube_on, bool axes_on);

    void printDetections();

    ArduinoCommands getOutputCommands(std::vector<TagPose> &tag_objects);

    GoalPose getMasterPose(std::vector<TagPose> &tag_objects);

    void setArduinoResponse(std::string received_msg);

    std::pair<bool, std::array<double, 3>> isValidResponse(std::string &msg);

    void rotatePose(GoalPose &pose, double angle);

    void convertToMotorSpeeds(ArduinoCommands &commands, double linear_speed, double ang_magnitude);

    void stopRobot();

private:
    std::shared_ptr<AprilTagDetector> mApriltagDetector;

    SimpleController linear_controller_;
    SimpleController angular_controller_;

    // PIDController linear_controller_;
    // PIDController angular_controller_;
    CameraController camera_angular_controller_;

    GoalPose goal_pose_;

    double current_linear_speed_;
    double current_angular_speed_;

    double last_controller_update_time_;

    double camera_yaw_;
    double goal_x_offset_;
    double goal_y_offset_;
    double prev_camera_yaw_;
};

#endif
