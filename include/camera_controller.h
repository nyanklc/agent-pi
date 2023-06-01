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

    int update(int current);

private:
    int goal_;
    double multiplier_;
    int tolerance_;
};

#endif