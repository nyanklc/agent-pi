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

class Agent {
   public:
    Agent();

    std::vector<TagPose> process(cv::Mat &frame);

    void drawDetections(cv::Mat &frame, bool cube_on, bool axes_on);

    void printDetections();

    ArduinoCommands getOutputCommands(std::vector<TagPose> &tag_objects);

    Transform getAgentToCameraTransform();

    Transform getTagToMaster(std::vector<TagPose> &tag_objects, size_t &index);

    void updateAgentToCameraTransform(double dyaw);

   private:

    std::shared_ptr<AprilTagDetector> mApriltagDetector;

    PIDController linear_controller_;
    PIDController angular_controller_;
    CameraController camera_angular_controller_;

    Transform agent_to_camera_initial_;
    Transform agent_to_camera_current_;
};

#endif
