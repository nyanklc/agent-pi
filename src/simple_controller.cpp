#include "../include/simple_controller.h"

void SimpleController::init(double step_amount, double tolerance, double min_limit)
{
    step_amount_ = step_amount;
    tolerance_ = tolerance;
    min_limit_ = min_limit;
}

double SimpleController::update(double measurement)
{
    if ((measurement < min_limit_ && measurement > 0)|| (measurement > - min_limit_ && measurement < 0))
        if (goal_ > min_limit_)
            return min_limit_ - measurement;
        else if (goal_ < -min_limit_)
            return -min_limit_ - measurement;
        else
            return -measurement;

    return (std::fabs(goal_ - measurement) > tolerance_) ? ((goal_ - measurement > 0) ? step_amount_ : -step_amount_) : 0;
}

void SimpleController::setGoal(double new_goal)
{
    goal_ = new_goal;
}