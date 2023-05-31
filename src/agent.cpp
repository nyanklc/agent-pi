#include "../include/agent.h"

Agent::Agent()
{
    mApriltagDetector = std::make_shared<AprilTagDetector>();
    mApriltagDetector->init();

    linear_controller_.init(LINEAR_P, LINEAR_I, LINEAR_D, 0, 0, 0, LINEAR_LIM_MAX, LINEAR_LIM_MIN);
    angular_controller_.init(ANGULAR_P, ANGULAR_I, ANGULAR_D, 0, 0, 0, ANGULAR_LIM_MAX, ANGULAR_LIM_MIN);
    camera_angular_controller_.init(CAMERA_SIZE_X / 2, 1, CAMERA_CONTROLLER_TOLERANCE);

    camera_yaw_ = CAMERA_YAW_INITIAL;

    current_linear_speed_ = 0;
    current_angular_speed_ = 0;
    last_controller_update_time_ = 0;
}

void Agent::drawDetections(cv::Mat &frame, bool cube_on, bool axes_on)
{
    if (!GUI_ON)
        return;

    mApriltagDetector->drawDetections(frame);
    mApriltagDetector->drawMarkers(frame, cube_on, axes_on);
}

void Agent::printDetections()
{
    std::vector<cv::Point> detectedPoints =
        mApriltagDetector->getDetectionPoints();
    for (size_t i = 0; i < detectedPoints.size(); i++)
        std::cout << "x" << i + 1 << ": " << detectedPoints[i].x << ", y" << i + 1
                  << ": " << detectedPoints[i].y << "\n";
}

std::vector<TagPose> Agent::process(cv::Mat &frame)
{
    return mApriltagDetector->process(frame);
}

void Agent::convertToMotorSpeeds(ArduinoCommands &commands, double linear_speed, double ang_magnitude)
{
    // commands.left_motor_speed = LIN_ANG_CONVERSION_LIN_MULTIPLIER * linear_speed + LIN_ANG_CONVERSION_ANG_MULTIPLIER * ang_magnitude;
    commands.left_motor_speed = 0;
    // commands.right_motor_speed = LIN_ANG_CONVERSION_LIN_MULTIPLIER * linear_speed - LIN_ANG_CONVERSION_ANG_MULTIPLIER * ang_magnitude;
    commands.right_motor_speed = 0;
}

ArduinoCommands Agent::getOutputCommands(std::vector<TagPose> &tag_objects)
{
    // camera control
    int tag_center_average = 0;
    for (auto &tag : tag_objects)
    {
        tag_center_average += tag.center_x_px;
    }
    tag_center_average /= tag_objects.size();

    goal_pose_ = getMasterPose(tag_objects);

    ArduinoCommands commands;
    // commands.camera_step_count = -camera_angular_controller_.update(tag_center_average); // negative to fix direction
    commands.camera_step_count = 0;

    // camera stuff
    camera_yaw_ -= commands.camera_step_count * CAMERA_STEP_ANGLE;
    while (camera_yaw_ > M_PI)
        camera_yaw_ -= 2 * M_PI;
    while (camera_yaw_ <= -M_PI)
        camera_yaw_ += 2 * M_PI;

    std::cout << "camera yaw: " << camera_yaw_ << std::endl;
    double rotation_angle = CAMERA_YAW_INITIAL - camera_yaw_;
    rotatePose(goal_pose_, -rotation_angle); // maybe -rotation_angle

    goal_pose_.print("MASTER POSE");

    goal_pose_.x -= GOAL_POSE_X_OFFSET;
    goal_pose_.y -= GOAL_POSE_Y_OFFSET;
    goal_pose_.print("MIMIC POSE");

    // calculate what current speeds should be
    double goal_vector_angle = std::atan2(goal_pose_.y, goal_pose_.x);
    double goal_vector_len = std::hypot(goal_pose_.x, goal_pose_.y);

    bool backward_enable = (goal_pose_.y < GOAL_POSE_Y_OFFSET) && goal_vector_len < LINEAR_BACKWARD_RADIUS;
    // bool backward_enable = false;
    // double lin_magnitude =  (backward_enable ? -1 : 1) * std::hypot(goal_pose_.x, goal_pose_.y) * LINEAR_GOAL_MULTIPLIER;

    // ulas modification (perpendicular distance to the goal)
    double lin_magnitude = (backward_enable ? -1 : 1) * (goal_vector_angle > M_PI / 4 ? std::fabs(goal_pose_.y) : std::fabs(goal_pose_.x)) * LINEAR_GOAL_MULTIPLIER;
    // double linear_speed = 0;
    double yaw_should_be = 0;
    // turn towards the mimic point if we're not close enough, otherwise turn towards master's orientation
    if (goal_vector_len < LINEAR_BACKWARD_RADIUS)
    {
        yaw_should_be = goal_pose_.yaw;
    }
    else
    {
        yaw_should_be = goal_vector_angle;
    }
    double ang_magnitude = (M_PI / 2 - yaw_should_be) * ANGULAR_GOAL_MULTIPLIER; // check if this should be negated

    // control current speeds
    // linear_controller_.setGoal(lin_magnitude);
    // linear_controller_.setGoal(0);
    // angular_controller_.setGoal(ang_magnitude);
    // angular_controller_.setGoal(0);
    // std::chrono::duration<double, std::micro> dur = std::chrono::high_resolution_clock::now();
    // double curr_time = dur.count();
    // current_linear_speed_ += linear_controller_.update(current_linear_speed_, curr_time - last_controller_update_time_);
    // current_angular_speed_ += angular_controller_.update(current_angular_speed_, curr_time - last_controller_update_time_);
    // last_controller_update_time_ = curr_time;  // idk about this

    current_linear_speed_ = lin_magnitude;
    current_angular_speed_ = ang_magnitude;

    std::cout << "current lin: " << current_linear_speed_ << ", current ang: " << current_angular_speed_ << std::endl;

    convertToMotorSpeeds(commands, current_linear_speed_, current_angular_speed_);

    commands.print("commands");

    return commands;
}

void Agent::rotatePose(GoalPose &pose, double angle)
{
    double newx = pose.x * std::cos(angle) - pose.y * std::sin(angle);
    double newy = pose.x * std::sin(angle) + pose.y * std::cos(angle);

    pose.x = newx;
    pose.y = newy;

    pose.yaw += angle;
    while (pose.yaw > M_PI)
        pose.yaw -= 2 * M_PI;
    while (pose.yaw <= -M_PI)
        pose.yaw += 2 * M_PI;
}

GoalPose Agent::getMasterPose(std::vector<TagPose> &tag_objects)
{
    auto tag = tag_objects[0];
    // std::cout << "BEFORE tagx: " << tag.x << " tagy: " << tag.y << " tagz: " << tag.z << " tagroll: " << tag.roll << " tagpitch: " << tag.pitch << " tagyaw: " << tag.yaw << std::endl;
    double pitch = -tag.pitch;
    pitch -= M_PI / 2;

    double x = tag.x;
    double y = tag.z;
    switch (tag.id)
    {
    case TAG_ID_1:
        pitch += M_PI;
        while (pitch > M_PI)
            pitch -= 2 * M_PI;
        while (pitch <= -M_PI)
            pitch += 2 * M_PI;
        break;
    case TAG_ID_2:
        pitch -= M_PI / 2;
        while (pitch > M_PI)
            pitch -= 2 * M_PI;
        while (pitch <= -M_PI)
            pitch += 2 * M_PI;
        break;
    case TAG_ID_3:
        while (pitch > M_PI)
            pitch -= 2 * M_PI;
        while (pitch <= -M_PI)
            pitch += 2 * M_PI;
        break;
    case TAG_ID_4:
        pitch += M_PI / 2;
        while (pitch > M_PI)
            pitch -= 2 * M_PI;
        while (pitch <= -M_PI)
            pitch += 2 * M_PI;
        break;
    }
    GoalPose gp;
    gp.x = x + TAG_BOX_SIZE / 2 * std::cos(pitch);
    gp.y = y + TAG_BOX_SIZE / 2 * std::sin(pitch);
    gp.yaw = pitch;
    return gp;
}

void Agent::setArduinoResponse(std::string received_msg)
{
    auto is_valid = isValidResponse(received_msg);
    if (!is_valid.first)
    {
        std::cout << "response not valid\n";
        return;
    }

    double lin_spd = is_valid.second[0];
    double ang_spd = is_valid.second[1];
    double cam_ang_spd = is_valid.second[2];

    std::cout << "lin_spd: " << lin_spd << ", ang_spd: " << ang_spd << ", cam_ang_spd: " << cam_ang_spd << "\n";
}

std::pair<bool, std::array<double, 3>> Agent::isValidResponse(std::string &msg)
{
    msg += ' ';
    std::stringstream ss(msg);
    std::string word;
    char count = 0;
    std::array<double, 3> arr;
    while (ss >> word)
    {
        std::cout << word << std::endl;
        if (count == 3)
        {
            count = 255;
            break;
        }
        arr[count] = (double)atof(word.c_str());
        count++;
    }
    bool valid = false;
    if (count == 3)
        valid = true;
    else if (count == 2)
    {
        std::string last_num = msg.substr(msg.size() - 5, 5);
        arr[2] = (double)atof(last_num.c_str());
        valid = true;
    }
    std::pair<bool, std::array<double, 3>> ret;
    ret.first = valid;
    ret.second = arr;
    return ret;
}
