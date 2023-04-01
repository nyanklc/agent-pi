#ifndef __APRIL_TAG_DETECTOR_H
#define __APRIL_TAG_DETECTOR_H

#include <math.h>

#include <iostream>
#include <memory>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "globals.h"
#include "master_object.h"
#include "tag_pose.h"
#include "utils.h"

extern "C" {
#include <apriltag/apriltag.h>
#include <apriltag/apriltag_math.h>
#include <apriltag/apriltag_pose.h>
#include <apriltag/common/getopt.h>
#include <apriltag/common/matd.h>
#include <apriltag/tagStandard41h12.h>
}

class AprilTagDetector {
   public:
    AprilTagDetector();

    void init(std::shared_ptr<MasterObject> master_obj);

    ~AprilTagDetector();

    std::vector<TagPose> process(cv::Mat &frame);

    bool findObject(cv::Mat &frame);

    bool poseEstimation(cv::Mat &frame);

    std::vector<apriltag_pose_t> getPoses();

    zarray *getDetections();

    void printPoses(std::vector<apriltag_pose_t> &poses);

    void printDetections(zarray *detections);

    void drawDetections(cv::Mat &frame);

    void drawMarkers(cv::Mat &frame, bool cube_on, bool axes_on);

    std::vector<cv::Point> getDetectionPoints();

   private:
    apriltag_family_t *mFamily;
    apriltag_detector_t *mDetector;
    zarray_t *mDetections;
    std::shared_ptr<MasterObject> mObj;
    apriltag_detection_info_t mInfo;
    std::vector<apriltag_pose_t> mPoses;
    std::vector<apriltag_pose_t> mHPoses;
    std::vector<std::pair<apriltag_pose_t, apriltag_pose_t>> mPosesOrthogonal;
};

#endif
