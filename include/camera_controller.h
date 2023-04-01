#ifndef __BANGBANG_H_
#define __BANGBANG_H_

#include <math.h>
#include <iostream>

class CameraController {
   public:
    CameraController();

    void init(double goal, double multiplier, double tolerance);

    double update(double current);

   private:
    double goal_;
    double multiplier_;
    double tolerance_;
};

#endif