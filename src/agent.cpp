#include "../include/agent.h"

Agent::Agent()
{
    mApriltagDetector = std::make_shared<AprilTagDetector>();
    mApriltagDetector->init();

    camera_yaw_ = CAMERA_YAW_INITIAL;

    current_linear_speed_ = 0;
    current_angular_speed_ = 0;
    last_controller_update_time_ = 0;

    goal_x_offset_ = 0;
    goal_y_offset_ = 0;

    goal_pose_.x = 0;
    goal_pose_.y = 0;
    goal_pose_.yaw = CAMERA_YAW_INITIAL;

    mimic_pose_.x = 0;
    mimic_pose_.y = 0;
    mimic_pose_.yaw = CAMERA_YAW_INITIAL;

    camera_angular_controller_.init(CAMERA_SIZE_X / 2, 1, CAMERA_CONTROLLER_TOLERANCE);

    // linear_controller_.init(LINEAR_STEP_AMOUNT, LINEAR_TOLERANCE, LINEAR_MIN_LIMIT);  // simple
    // angular_controller_.init(ANGULAR_STEP_AMOUNT, ANGULAR_TOLERANCE, ANGULAR_MIN_LIMIT);  // simple
    // linear_controller_.init(LINEAR_P, LINEAR_I, LINEAR_D, 0, 0, 0, LINEAR_LIM_MIN, LINEAR_LIM_MAX);
    // angular_controller_.init(ANGULAR_P, ANGULAR_I, ANGULAR_D, 0, 0, 0, ANGULAR_LIM_MIN, ANGULAR_LIM_MAX);

    state_ = AgentState::SEARCH;

    wait_counter_ = 0;
    search_camera_direction_ = false;
    search_outside_once_ = false;
    search_start_angle_ = CAMERA_YAW_INITIAL;
}

// void Agent::debugPIDOutput()
// {
//     myPIDController.getProportionalComponent();
//     myPIDController.getIntegralComponent();
//     myPIDController.getDerivativeComponent();
//     myPIDController.getP(); // Returns P Gain
//     myPIDController.getI(); // Returns I Gain
//     myPIDController.getD(); // Returns D Gain
//     myPIDController.getTarget();
//     myPIDController.getFeedback();
//     myPIDController.getOutput();
//     myPIDController.getError();
// }


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

void Agent::convertToMotorSpeeds(ArduinoCommands &commands, float linear_speed, float ang_magnitude)
{
    float linear = LIN_ANG_CONVERSION_LIN_MULTIPLIER * linear_speed;
    float angular = LIN_ANG_CONVERSION_ANG_MULTIPLIER * ang_magnitude;

    if (linear < 0)
        angular *= -1;

    commands.left_motor_speed = linear + angular;
    commands.right_motor_speed = linear - angular;

    if (commands.left_motor_speed > 255)
        commands.left_motor_speed = 255;
    else if (commands.left_motor_speed < -255)
        commands.left_motor_speed = -255;

    if (commands.right_motor_speed > 255)
        commands.right_motor_speed = 255;
    else if (commands.right_motor_speed < -255)
        commands.right_motor_speed = -255;

    // eren scale modification
    if (commands.left_motor_speed > 0)
    {
        int scaled_left = MIN_MOTOR_ANALOG + (int)(((float)commands.left_motor_speed / (float)MAX_MOTOR_ANALOG) * (float)(MAX_MOTOR_ANALOG-MIN_MOTOR_ANALOG));
        if (scaled_left >= MIN_MOTOR_ANALOG && scaled_left < MIN_MOTOR_ANALOG)
            scaled_left = 0;
        commands.left_motor_speed = scaled_left;
    }
    else
    {
        int scaled_left = -MIN_MOTOR_ANALOG + (int)(((float)commands.left_motor_speed / (float)MAX_MOTOR_ANALOG) * (float)(MAX_MOTOR_ANALOG-MIN_MOTOR_ANALOG));
        if (scaled_left <= -MIN_MOTOR_ANALOG && scaled_left > -MIN_MOTOR_ANALOG)
            scaled_left = 0;
        commands.left_motor_speed = scaled_left;
    }
    if (commands.right_motor_speed > 0)
    {
        int scaled_right = MIN_MOTOR_ANALOG + (int)(((float)commands.right_motor_speed / (float)MAX_MOTOR_ANALOG) * (float)(MAX_MOTOR_ANALOG-MIN_MOTOR_ANALOG));
        if (scaled_right >= MIN_MOTOR_ANALOG && scaled_right < MIN_MOTOR_ANALOG)
            scaled_right = 0;
        commands.right_motor_speed = scaled_right;
    }
    else
    {
        int scaled_right = -MIN_MOTOR_ANALOG + (int)(((float)commands.right_motor_speed / (float)MAX_MOTOR_ANALOG) * (float)(MAX_MOTOR_ANALOG-MIN_MOTOR_ANALOG));
        if (scaled_right <= -MIN_MOTOR_ANALOG && scaled_right > -MIN_MOTOR_ANALOG)
            scaled_right = 0;
        commands.right_motor_speed = scaled_right; 
    }
}

/* keep mimic pose offsets constant in the global frame, adjusts offsets base on the camera rotation */
void Agent::_rotateGoalOffsets(float angle, float &x, float &y, float prev_angle)
{
    // std::cout << "angle: " << angle << ", prev_angle: " << prev_angle << std::endl;

    if (prev_angle == angle)
        return;

    float newx = x * std::cos(angle - prev_angle) - y * std::sin(angle - prev_angle);
    float newy = x * std::sin(angle - prev_angle) + y * std::cos(angle - prev_angle);

    x = newx;
    y = newy;
}

void Agent::stopRobot()
{
    current_linear_speed_ = 0;
    current_angular_speed_ = 0;
}

void Agent::_setGoalOffsets(float new_x_offset, float new_y_offset)
{
    goal_x_offset_ = new_x_offset;
    goal_y_offset_ = new_y_offset;
}

void Agent::_updateCameraAngle(int count)
{
    prev_camera_yaw_ = camera_yaw_;
    camera_yaw_ -= count * CAMERA_STEP_ANGLE;
    while (camera_yaw_ > M_PI)
        camera_yaw_ -= 2 * M_PI;
    while (camera_yaw_ <= -M_PI)
        camera_yaw_ += 2 * M_PI;

    std::cout << "camera yaw: " << camera_yaw_ << std::endl;
}

/* tracks center average of the detected tags, adjusts current camera angle */
void Agent::_controlCamera(std::vector<TagPose> &tag_objects, ArduinoCommands &commands)
{
    // camera control
    int tag_center_average = 0;
    for (auto &tag : tag_objects)
    {
        std::cout << std::endl;
        tag_center_average += tag.center_x_px;
    }
    tag_center_average /= tag_objects.size();
    // std::cout << "tag center: " << tag_center_average << std::endl;

    commands.camera_step_count = -camera_angular_controller_.update(tag_center_average, std::hypot(tag_objects[0].x, tag_objects[0].y)); // negative to fix direction

    _updateCameraAngle(commands.camera_step_count);
}

/* adjusts master pose based on the current camera angle */
void Agent::_rotateMasterPose()
{
    float rotation_angle = CAMERA_YAW_INITIAL - camera_yaw_;
    rotatePose(goal_pose_, -rotation_angle); // maybe -rotation_angle
}

/* converts master pose to mimic pose, uses offsets */
void Agent::_getMimicPose()
{
    goal_pose_.print("MASTER POSE");
    mimic_pose_.x = goal_pose_.x - goal_x_offset_;
    mimic_pose_.y = goal_pose_.y - goal_y_offset_;
    mimic_pose_.yaw = goal_pose_.yaw;
    mimic_pose_.print("MIMIC POSE");
}

/* outputs commands to be sent to arduino, based on the current state of Agent */
ArduinoCommands Agent::getOutputCommands(std::vector<TagPose> &tag_objects)
{
    ArduinoCommands commands;

    std::cout << "STATE: " << state_ << std::endl;

    switch (state_)
    {
        case AgentState::SEARCH:
        {
            commands = getZeroCommand();
            if (tag_objects.size() > 0)
            {
                state_ = AgentState::TRACK_MIMIC;
                last_state_ = AgentState::SEARCH;

                // select goal offset (mimic pose)
                goal_pose_ = getMasterPose(tag_objects);
                _rotateMasterPose();  // adjust with camera angle
                float angle = std::atan2(goal_pose_.y, goal_pose_.x);
                goal_x_offset_ = std::cos(angle) * GOAL_POSE_OFFSET;
                goal_y_offset_ = std::sin(angle) * GOAL_POSE_OFFSET;

                std::cout << "calculated new offsets, x:" << goal_x_offset_ << ", y:" << goal_y_offset_ << std::endl;

                break;
            }
            if (std::fabs(camera_yaw_ - search_start_angle_) > SEARCH_CAMERA_TOLERANCE)
                search_outside_once_ = true;

            if (search_outside_once_)
                if (std::fabs(camera_yaw_ - search_start_angle_) < SEARCH_CAMERA_TOLERANCE)
                    search_camera_direction_ = !search_camera_direction_;

            commands.camera_step_count = search_camera_direction_ ? 1 : -1;
            _updateCameraAngle(commands.camera_step_count);

        }
        break;

        case AgentState::TRACK_MIMIC:
        {
            if (tag_objects.size() == 0)
            {
                commands = getZeroCommand();
                state_ = AgentState::WAIT_COUNTER;
                last_state_ = AgentState::TRACK_MIMIC;
                break;
            }

            _controlCamera(tag_objects, commands);
            goal_pose_ = getMasterPose(tag_objects);  // camera frame
            _rotateMasterPose();  // carry to agent frame
            _getMimicPose();
            _rotateGoalOffsets(camera_yaw_ - CAMERA_YAW_INITIAL, goal_x_offset_, goal_y_offset_, prev_camera_yaw_ - CAMERA_YAW_INITIAL);

            std::cout << "goal_x_offset: " << goal_x_offset_ << ", goal_y_offset: " << goal_y_offset_ << std::endl;
            // calculate what current speeds should be
            float goal_vector_angle = std::atan2(mimic_pose_.y, mimic_pose_.x);
            float goal_vector_len = std::hypot(mimic_pose_.x, mimic_pose_.y);

            std::cout << "goal_Vector_angle: " << goal_vector_angle << std::endl;
            std::cout << "goal_vector_len: " << goal_vector_len << std::endl;

            // ulas modification (perpendicular distance to the goal)
            float lin_magnitude = mimic_pose_.y;
            // float linear_speed = 0;

            float yaw_should_be = 0;
            // turn towards the mimic point if we're not close enough, otherwise turn towards master's orientation
            if (std::fabs(mimic_pose_.y) < MIMIC_RADIUS)
            {
                // std::cout << "yaw should be towards master's orientation\n";
                yaw_should_be = mimic_pose_.yaw;
            }
            else
            {
                // std::cout << "yaw should be towards mimic point\n";
                yaw_should_be = goal_vector_angle;
            }
            float ang_magnitude = (M_PI / 2 - yaw_should_be); // check if this should be negated
            while (ang_magnitude > M_PI)
                ang_magnitude -= 2 * M_PI;
            while (ang_magnitude < -M_PI)
                ang_magnitude += 2 * M_PI;

            // std::cout << "yaw should be: " << yaw_should_be << std::endl;

            if (std::fabs(ang_magnitude) < 0.5)
                ang_magnitude = 0;

            // controllers
            // TODO: use PID controllers to update current linear and angular speeds
            convertToMotorSpeeds(commands, lin_magnitude, ang_magnitude);

            // test
            std::cout << "linmag: " << lin_magnitude << ", angmag: " << ang_magnitude << std::endl;
            commands.print("commands");

        }
        break;

        case AgentState::WAIT_COUNTER:
        {
            if (tag_objects.size() > 0)
            {
                state_ = AgentState::TRACK_MIMIC;
                last_state_ = AgentState::WAIT_COUNTER;
                break;
            }
            wait_counter_++;
            commands = getZeroCommand();
            if (wait_counter_ > WAIT_COUNTER_LIMIT)
            {
                state_ = AgentState::SEARCH;
                search_outside_once_ = false;
                search_start_angle_ = camera_yaw_;
                wait_counter_ = 0;
            }
        }
        break;
    }

    // commands = getZeroCommand();
    commands.left_motor_speed = 0;
    commands.right_motor_speed = 0;
    return commands;
}

void Agent::rotatePose(GoalPose &pose, float angle)
{
    float newx = pose.x * std::cos(angle) - pose.y * std::sin(angle);
    float newy = pose.x * std::sin(angle) + pose.y * std::cos(angle);

    pose.x = newx;
    pose.y = newy;

    pose.yaw += angle;
    while (pose.yaw > M_PI)
        pose.yaw -= 2 * M_PI;
    while (pose.yaw <= -M_PI)
        pose.yaw += 2 * M_PI;
}

/* returns master's pose in the camera frame */
GoalPose Agent::getMasterPose(std::vector<TagPose> &tag_objects)
{
    auto tag = tag_objects[0];
    float pitch = -tag.pitch;
    pitch -= M_PI / 2;

    float x = tag.x;
    float y = tag.z;
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
    // TODO
    gp.x = x; // + TAG_BOX_SIZE / 2 * std::cos(pitch);
    gp.y = y; // + TAG_BOX_SIZE / 2 * std::sin(pitch);
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

    float lin_spd = is_valid.second[0];
    float ang_spd = is_valid.second[1];
    float cam_ang_spd = is_valid.second[2];

    std::cout << "lin_spd: " << lin_spd << ", ang_spd: " << ang_spd << ", cam_ang_spd: " << cam_ang_spd << "\n";
}

std::pair<bool, std::array<float, 3>> Agent::isValidResponse(std::string &msg)
{
    msg += ' ';
    std::stringstream ss(msg);
    std::string word;
    char count = 0;
    std::array<float, 3> arr;
    while (ss >> word)
    {
        std::cout << word << std::endl;
        if (count == 3)
        {
            count = 255;
            break;
        }
        arr[count] = (float)atof(word.c_str());
        count++;
    }
    bool valid = false;
    if (count == 3)
        valid = true;
    else if (count == 2)
    {
        std::string last_num = msg.substr(msg.size() - 5, 5);
        arr[2] = (float)atof(last_num.c_str());
        valid = true;
    }
    std::pair<bool, std::array<float, 3>> ret;
    ret.first = valid;
    ret.second = arr;
    return ret;
}
