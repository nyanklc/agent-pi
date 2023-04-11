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

    current_yaw_ = 0;
    current_linear_speed_ = 0;
    current_angular_speed_ = 0;
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
    Transform camera_to_master = getCameraToMaster(tag_objects);
    Transform agent_to_master;
    agent_to_master.T = agent_to_camera_current_.T * camera_to_master.T;

    printTransform(camera_to_master, "camera_to_master: ");

    // get the position/orientation we want to be at
    goal_pose_ = getGoalPose(agent_to_master);
    goal_pose_.print("goal pose");

    // set controller goals
    double linear_goal = std::sqrt(std::pow(goal_pose_.x, 2) + std::pow(goal_pose_.y, 2)) * LINEAR_GOAL_MULTIPLIER;
    double angular_goal = getAngularDifference(goal_pose_.yaw, current_yaw_) * ANGULAR_GOAL_MULTIPLIER;
    linear_controller_.setGoal(linear_goal); // TODO: sqrt is slow
    angular_controller_.setGoal(angular_goal);

    std::cout << "linear_goal: " << linear_goal << "\n";
    std::cout << "angular_goal: " << angular_goal << "\n";

    ArduinoCommands commands;
    auto durat = (std::chrono::system_clock::now() - last_controller_update_time_);
    double dt =  durat.count() * 1e-9;
    last_controller_update_time_ = std::chrono::system_clock::now();

    commands.angular_speed = linear_controller_.update(current_linear_speed_, dt);
    commands.linear_speed = angular_controller_.update(current_angular_speed_, dt);
    commands.camera_angular_speed = camera_angular_controller_.update(tag_center_average);

    return commands;
}

Transform Agent::getCameraToMaster(std::vector<TagPose> &tag_objects) {
    // TODO: implement
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

GoalPose Agent::getGoalPose(Transform &agent_to_master) {
    auto R = getRotationFromTransform(agent_to_master);
    auto t = getTranslationFromTransform(agent_to_master);

    auto rpy = getRPY(R);

    auto goal_position = truncateVector(t, 3/4);

    std::cout << "roll: " << rpy[0] << "\n";
    std::cout << "pitch: " << rpy[1] << "\n";
    std::cout << "yaw: " << rpy[2] << "\n";

    GoalPose goal_pose;
    goal_pose.x = goal_position.at<double>(0, 0);
    goal_pose.y = goal_position.at<double>(1, 0);
    goal_pose.yaw = rpy[2]; // TODO: which angle is the correct "yaw"?

    return goal_pose;
}

void Agent::setArduinoResponse(std::string received_msg) {
    // TODO: set current linear/angular speed
    char lin_str[4] = {received_msg[0], received_msg[1], received_msg[2], received_msg[3]};
    char ang_str[4] = {received_msg[0], received_msg[1], received_msg[2], received_msg[3]};
    // TODO: camera speed
    // std::string lin_str_str(lin_str);
    // std::string ang_str_str(ang_str);
    // TODO: set current linear and angular speeds
}