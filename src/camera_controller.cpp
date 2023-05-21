#include "../include/camera_controller.h"

CameraController::CameraController() {}

void CameraController::init(int goal, int multiplier, int tolerance) {
    goal_ = goal;
    multiplier_ = multiplier;
    tolerance_ = tolerance;
}

double CameraController::update(int current) {
    // std::cout << "goal: " << goal_ << ", current: " << current << std::endl;
    if (std::abs(goal_ - current) <= tolerance_)
        return 0;
    else
    {
        int val = (goal_ - current) * multiplier_;
        if (val > CAMERA_CONTROLLER_LIMIT)
            val = CAMERA_CONTROLLER_LIMIT;
        else if (val < -CAMERA_CONTROLLER_LIMIT)
            val = -CAMERA_CONTROLLER_LIMIT;
        return val;
    }
}
