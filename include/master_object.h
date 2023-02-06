#ifndef __MASTER_OBJECT_H
#define __MASTER_OBJECT_H

#include <vector>
#include <algorithm>

#include <opencv2/opencv.hpp>

class MasterObject {
    void setPosition(std::vector<cv::Point> corners);

private:
    double cx;
    double cy;
};

#endif
