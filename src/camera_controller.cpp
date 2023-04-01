#include "../include/camera_controller.h"

CameraController::CameraController() {}

void CameraController::init(double goal, double multiplier, double tolerance) {
    goal_ = goal;
    multiplier_ = multiplier;
    tolerance_ = tolerance;
}

double CameraController::update(double current) {
    if (std::fabs(goal_ - current) <= tolerance_)
        return 0;
    else
        return (goal_ - current) * multiplier_;
}
