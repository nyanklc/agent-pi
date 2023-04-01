#include "../include/agent.h"

Agent::Agent() {
    mApriltagDetector = std::make_shared<AprilTagDetector>();
    mApriltagDetector->init();

    linear_controller_.init(LINEAR_P, LINEAR_I, LINEAR_D, 0, 0, 0, LINEAR_LIM_MAX, LINEAR_LIM_MIN);
    angular_controller_.init(ANGULAR_P, ANGULAR_I, ANGULAR_D, 0, 0, 0, ANGULAR_LIM_MAX, ANGULAR_LIM_MIN);
    // TODO:
    camera_angular_controller_.init(CAMERA_SIZE_X / 2, CAMERA_CONTROLLER_MULTIPLIER, CAMERA_CONTROLLER_TOLERANCE);

    agent_to_camera_initial_ = getAgentToCameraTransform();
    printTransform(agent_to_camera_initial_, "initial agent to camera");
    agent_to_camera_current_ = agent_to_camera_initial_;
     printTransform(agent_to_camera_initial_, "initial current");
}

void Agent::drawDetections(cv::Mat &frame, bool cube_on, bool axes_on) {
    if (!GUI_ON)
        return;

    mApriltagDetector->drawDetections(frame);
    mApriltagDetector->drawMarkers(frame, cube_on, axes_on);
}

void Agent::printDetections() {
    std::vector<cv::Point> detectedPoints =
        mApriltagDetector->getDetectionPoints();
    for (size_t i = 0; i < detectedPoints.size(); i++)
        std::cout << "x" << i + 1 << ": " << detectedPoints[i].x << ", y" << i + 1
                  << ": " << detectedPoints[i].y << "\n";
}

std::vector<TagPose> Agent::process(cv::Mat &frame) {
    return mApriltagDetector->process(frame);
}

ArduinoCommands Agent::getOutputCommands(std::vector<TagPose> tag_objects) {
    // camera control
    int tag_center_average = 0;
    for (auto &tag : tag_objects) {
        tag_center_average += tag.center_x_px;
    }
    tag_center_average /= tag_objects.size();
    tag_center_average *= 2;  // since we resized the image

    // TODO: linear/angular control

    ArduinoCommands commands;
    commands.angular_speed = 1.11;
    commands.linear_speed = 2.22;
    commands.camera_angular_speed = camera_angular_controller_.update(tag_center_average);

    return commands;
}

Transform Agent::getAgentToCameraTransform() {
    cv::Mat R = getRotationMatrix(AGENT_TO_CAMERA_ROLL_INITIAL, AGENT_TO_CAMERA_PITCH_INITIAL, AGENT_TO_CAMERA_YAW_INITIAL);
    cv::Mat t = getTranslationMatrix(AGENT_TO_CAMERA_X_OFFSET, AGENT_TO_CAMERA_Y_OFFSET, AGENT_TO_CAMERA_Z_OFFSET);
    Transform tf = constructTransform(R, t);
    return tf;
}

void Agent::updateAgentToCameraTransform(double dyaw) {
    std::cout << "here1\n";
    cv::Mat R_curr = getRotationFromTransform(agent_to_camera_current_);
    std::cout << "here2\n";
    R_curr = getRotationMatrix(0, 0, dyaw) * R_curr;
    std::cout << "here3\n";
    agent_to_camera_current_ = constructTransform(R_curr, getTranslationFromTransform(agent_to_camera_current_));
    std::cout << "here4\n";
    printTransform(agent_to_camera_current_, "updated current");
}
