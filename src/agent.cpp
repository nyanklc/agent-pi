#include "../include/agent.h"

Agent::Agent() {
    mApriltagDetector = std::make_shared<AprilTagDetector>();
    mApriltagDetector->init();

    linear_controller_.init(LINEAR_P, LINEAR_I, LINEAR_D, 0, 0, 0, LINEAR_LIM_MAX, LINEAR_LIM_MIN);
    angular_controller_.init(ANGULAR_P, ANGULAR_I, ANGULAR_D, 0, 0, 0, ANGULAR_LIM_MAX, ANGULAR_LIM_MIN);
    // TODO:
    camera_angular_controller_.init(CAMERA_SIZE_X / 2, CAMERA_CONTROLLER_MULTIPLIER, CAMERA_CONTROLLER_TOLERANCE);

    agent_to_camera_initial_ = getAgentToCameraTransform();
    agent_to_camera_current_ = agent_to_camera_initial_;
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

ArduinoCommands Agent::getOutputCommands(std::vector<TagPose> &tag_objects) {
    // camera control
    int tag_center_average = 0;
    for (auto &tag : tag_objects) {
        tag_center_average += tag.center_x_px;
    }
    tag_center_average /= tag_objects.size();
    tag_center_average *= 2;  // since we resized the image

    // find the master's relative position to agent
    size_t tag_objects_index = 0;
    Transform tag_to_master = getTagToMaster(tag_objects, tag_objects_index);
    Transform camera_to_tag = constructTransform(getRotationMatrix(tag_objects[tag_objects_index].roll, tag_objects[tag_objects_index].pitch, tag_objects[tag_objects_index].yaw), getTranslationMatrix(tag_objects[tag_objects_index].x, tag_objects[tag_objects_index].y, tag_objects[tag_objects_index].z));
    Transform agent_to_master;
    agent_to_master.T = agent_to_camera_current_.T * camera_to_tag.T * tag_to_master.T;
    printTransform(agent_to_master, "agent_to_master");
    // TODO: linear/angular control

    ArduinoCommands commands;
    commands.angular_speed = 1.11;
    commands.linear_speed = 2.22;
    commands.camera_angular_speed = camera_angular_controller_.update(tag_center_average);

    return commands;
}

Transform Agent::getTagToMaster(std::vector<TagPose> &tag_objects, size_t &index) {
    // TODO: implement, put whichever tag_objects index you used to calculate the master's tf to the index
    index = 0;
    return constructTransform(getRotationMatrix(tag_objects[0].roll, tag_objects[0].pitch, tag_objects[0].yaw), getTranslationMatrix(tag_objects[0].x, tag_objects[0].y, tag_objects[0].z));
}

Transform Agent::getAgentToCameraTransform() {
    cv::Mat R = getRotationMatrix(AGENT_TO_CAMERA_ROLL_INITIAL, AGENT_TO_CAMERA_PITCH_INITIAL, AGENT_TO_CAMERA_YAW_INITIAL);
    cv::Mat t = getTranslationMatrix(AGENT_TO_CAMERA_X_OFFSET, AGENT_TO_CAMERA_Y_OFFSET, AGENT_TO_CAMERA_Z_OFFSET);
    Transform tf = constructTransform(R, t);
    return tf;
}

void Agent::updateAgentToCameraTransform(double dyaw) {
    cv::Mat R_curr = getRotationFromTransform(agent_to_camera_current_);
    R_curr = getRotationMatrix(0, 0, dyaw) * R_curr;
    agent_to_camera_current_ = constructTransform(R_curr, getTranslationFromTransform(agent_to_camera_current_));
    // printTransform(agent_to_camera_current_, "updated current");
}
