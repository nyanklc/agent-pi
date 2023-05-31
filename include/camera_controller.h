#ifndef __BANGBANG_H_
#define __BANGBANG_H_

#include <math.h>
#include <iostream>
#include "./globals.h"

class CameraController
{
public:
    CameraController();

    void init(int goal, int multiplier, int tolerance);

    double update(int current);

private:
    double goal_;
    double multiplier_;
    double tolerance_;
};

#endif