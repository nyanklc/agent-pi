#include "../include/pid.h"

#include <iostream>

PIDController::PIDController() {}

void PIDController::init(double p, double i, double d, double goal_value, double last_error, double last_measurement, double lim_min, double lim_max)
{
    kp_ = p;
    ki_ = i;
    kd_ = d;

    lim_min_ = lim_min;
    lim_max_ = lim_max;

    goal_ = goal_value;

    last_error_ = last_error;
    last_measurement_ = last_measurement;

    integrator_ = 0;
    differentiator_ = 0;

    out_ = 0;
}

double PIDController::update(double measurement, double dt)
{
    double err = goal_ - measurement;
    // std::cout << "GOAL: " << goal_ << std::endl;
    // std::cout << "MEAS: " << measurement << std::endl;
    // std::cout << "ERR: " << err << std::endl;

    // proportional term
    double proportional = kp_ * err;
    // std::cout << "PROPROOPR: " << proportional << std::endl;

    // filtered integral term
    double integral = integrator_ + 0.5 * ki_ * dt * (err - last_error_);

    // clamp integral term (anti-windup via dynamic integrator clamping)
    double lim_min_int;
    double lim_max_int;
    if (lim_max_ > proportional)
    {
        lim_max_int = lim_max_ - proportional;
    }
    else
    {
        lim_max_int = 0;
    }
    if (lim_min_ < proportional)
    {
        lim_min_int = lim_min_ - proportional;
    }
    else
    {
        lim_min_int = 0;
    }
    if (integral > lim_max_int)
    {
        integral = lim_max_int;
    }
    else if (integral < lim_min_int)
    {
        integral = lim_min_int;
    }

    // derivative term (bandlimited differentiator) (derivative on measurement to prevent kick)
    // TODO: tau?
    tau_ = 0.02;
    double derivative = -(-2.0 * kd_ * (measurement - last_measurement_) + (2.0 * tau_ - dt) * differentiator_) / (2.0 * tau_ * dt);

    out_ = proportional + integral + derivative;

    // limit the output
    if (out_ > lim_max_)
        out_ = lim_max_;
    else if (out_ < lim_min_)
        out_ = lim_min_;

    // memory
    last_error_ = err;
    last_measurement_ = measurement;
    integrator_ = integral;
    differentiator_ = derivative;

    // std::cout << "p: " << proportional << ", i: " << integral << ", d: " << derivative << ", output: " << out_ << std::endl;

    return out_;
}

void PIDController::setGoal(double new_goal)
{
    goal_ = new_goal;
}

double PIDController::getGoal()
{
    return goal_;
}