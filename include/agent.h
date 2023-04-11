#ifndef __AGENT_H
#define __AGENT_H

#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>

#include "april_tag_detector.h"
#include "arduino_commands.h"
#include "camera_calibrator.h"
#include "controls.h"
#include "globals.h"
#include "master_object.h"
#include "pid.h"
#include "camera_controller.h"

struct GoalPose {
    double x;
    double y;
    double yaw;

    void print(std::string name) {
        std::cout << name << " x: " << x << "y: " << y << " yaw: " << yaw << "\n";
    }
};

class Agent {
   public:
    Agent();

    std::vector<TagPose> process(cv::Mat &frame);

    void drawDetections(cv::Mat &frame, bool cube_on, bool axes_on);

    void printDetections();

    ArduinoCommands getOutputCommands(std::vector<TagPose> &tag_objects);

    Transform getAgentToCameraTransform();

    Transform getCameraToMaster(std::vector<TagPose> &tag_objects);

    void updateAgentToCameraTransform(double dyaw);

    GoalPose getGoalPose(Transform &agent_to_master);

    void setArduinoResponse(std::string received_msg);

   private:

    std::shared_ptr<AprilTagDetector> mApriltagDetector;

    PIDController linear_controller_;
    PIDController angular_controller_;
    CameraController camera_angular_controller_;

    Transform agent_to_camera_initial_;
    Transform agent_to_camera_current_;

    GoalPose goal_pose_;
    double current_yaw_;

    double current_linear_speed_;
    double current_angular_speed_;

    std::chrono::time_point<std::chrono::system_clock> last_controller_update_time_;
};

#endif
