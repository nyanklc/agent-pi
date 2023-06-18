#ifndef __AGENT_H
#define __AGENT_H

#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <bits/stdc++.h>
#include <chrono>

#include "april_tag_detector.h"
#include "arduino_commands.h"
#include "globals.h"
#include "camera_controller.h"
#include "simple_controller.h"
// #include "pid.h"  // old pid

static ArduinoCommands getZeroCommand()
{
    ArduinoCommands com;
    com.camera_step_count = 0;
    com.left_motor_speed = 0;
    com.right_motor_speed = 0;
    return com;
}

struct GoalPose
{
    float x;
    float y;
    float yaw;

    void print(std::string name)
    {
        std::cout << name << " x: " << x << " y: " << y << " yaw: " << yaw << "\n";
    }
};

enum AgentState {
    SEARCH,
    TRACK_MIMIC,
    WAIT_COUNTER
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

    std::pair<bool, std::array<float, 3>> isValidResponse(std::string &msg);

    void rotatePose(GoalPose &pose, float angle);

    void convertToMotorSpeeds(ArduinoCommands &commands, float linear_speed, float ang_magnitude);

    void stopRobot();

private:
    void _controlCamera(std::vector<TagPose> &tag_objects, ArduinoCommands &commands);

    void _rotateMasterPose();

    void _getMimicPose();

    void _rotateGoalOffsets(float angle, float &x, float &y, float prev_angle);

    void _setGoalOffsets(float new_x_offset, float new_y_offset);

    void _updateCameraAngle(int count);

    std::shared_ptr<AprilTagDetector> mApriltagDetector;

    // SimpleController linear_controller_;
    // SimpleController angular_controller_;

    CameraController camera_angular_controller_;

    GoalPose goal_pose_;
    GoalPose mimic_pose_;

    float current_linear_speed_;
    float current_angular_speed_;

    float last_controller_update_time_;

    float camera_yaw_;
    float goal_x_offset_;
    float goal_y_offset_;
    float prev_camera_yaw_;

    AgentState state_;
    AgentState last_state_;
    
    int wait_counter_;
    bool search_camera_direction_;
    bool search_outside_once_;
    float search_start_angle_;
};

#endif
