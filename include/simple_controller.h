#ifndef SIMPLE_CONTROLLER_H_
#define SIMPLE_CONTROLLER_H_

#include <iostream>
#include <math.h>

class SimpleController
{
public:
    SimpleController(){}
    
    void init(double step_amount, double tolerance, double min_limit);

    double update(double measurement);

    void setGoal(double new_goal);

private:
    double goal_;
    double step_amount_;
    double tolerance_;
    double min_limit_;
};

#endif