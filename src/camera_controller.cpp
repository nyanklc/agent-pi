#include "../include/camera_controller.h"

CameraController::CameraController() {}

void CameraController::init(int goal, int multiplier, int tolerance)
{
    goal_ = goal;
    multiplier_ = multiplier;
    tolerance_ = tolerance;
}

double CameraController::update(int current)
{
    // std::cout << "goal: " << goal_ << ", current: " << current << std::endl;
    if (std::abs(goal_ - current) <= CAMERA_CONTROLLER_TOLERANCE)
    {
        return 0;
    }
    else
    {
        int val = (goal_ - current) - ((goal_ - current) < 0 ? -1 : 1) * CAMERA_CONTROLLER_TOLERANCE;
        val *= CAMERA_CONTROLLER_MULTIPLIER;
        if (val > CAMERA_CONTROLLER_LIMIT)
            val = CAMERA_CONTROLLER_LIMIT;
        else if (val < -CAMERA_CONTROLLER_LIMIT)
            val = -CAMERA_CONTROLLER_LIMIT;

        return val;
    }
}
