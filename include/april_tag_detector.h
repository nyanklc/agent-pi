#ifndef __APRIL_TAG_DETECTOR_H
#define __APRIL_TAG_DETECTOR_H

#include "globals.h"
#include "master_object.h"

#include <string>
#include <iostream>
#include <memory>

#include <opencv2/opencv.hpp>

extern "C" {
#include <apriltag/apriltag.h>
#include <apriltag/tagStandard41h12.h>
#include <apriltag/common/getopt.h>
#include <apriltag/apriltag_math.h>
#include <apriltag/apriltag_pose.h>
}

class AprilTagDetector {
public:
    AprilTagDetector();

    void init(std::shared_ptr<MasterObject> master_obj);

    ~AprilTagDetector();

    bool findObject(cv::Mat &frame);

    zarray *getDetections();

    void drawDetections(cv::Mat &frame);

private:
    apriltag_family_t *mFamily;
    apriltag_detector_t *mDetector;
    zarray_t *mDetections;
    std::shared_ptr<MasterObject> mObj;
};


#endif
