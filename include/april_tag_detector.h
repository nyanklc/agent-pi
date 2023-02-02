#ifndef __APRIL_TAG_DETECTOR_H
#define __APRIL_TAG_DETECTOR_H

#include "globals.h"
#include "master_object.h"

#include <string>
#include <iostream>
#include <memory>
#include <vector>

#include <opencv2/opencv.hpp>

extern "C"
{
#include <apriltag/apriltag.h>
#include <apriltag/tagStandard41h12.h>
#include <apriltag/common/getopt.h>
#include <apriltag/apriltag_math.h>
#include <apriltag/apriltag_pose.h>
#include <apriltag/common/matd.h>
}

class AprilTagDetector
{
public:
    AprilTagDetector();

    void init(std::shared_ptr<MasterObject> master_obj);

    ~AprilTagDetector();

    bool findObject(cv::Mat &frame);

    bool poseEstimation(cv::Mat &frame);

    std::vector<apriltag_pose_t> getPoses();

    zarray *getDetections();

    void printPoses(std::vector<apriltag_pose_t> &poses);

    void drawDetections(cv::Mat &frame);

    std::vector<cv::Point> getDetectionPoints();

private:
    apriltag_family_t *mFamily;
    apriltag_detector_t *mDetector;
    zarray_t *mDetections;
    std::shared_ptr<MasterObject> mObj;
    apriltag_detection_info_t mInfo;
    std::vector<apriltag_pose_t> mPoses;
};

#endif
