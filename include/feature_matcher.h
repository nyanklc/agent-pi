#ifndef __FEATURE_MATCHER_H
#define __FEATURE_MATCHER_H

#include <memory>
#include <opencv2/opencv.hpp>
#include <vector>

#include "globals.h"
#include "master_object.h"

struct FrameData {
    std::vector<cv::KeyPoint> kp;
    cv::Mat des;
};

class FeatureMatcher {
   public:
    FeatureMatcher();

    // virtual const.
    void init(std::string ref_img_path, int knn_k);

    void getReferenceFrame(std::string img_path);

    bool findObject(cv::Mat &frame);

    void match(cv::Mat &des, int k);

    void setObj(std::shared_ptr<MasterObject> m);

    void drawDetections(cv::Mat &frame);

    cv::Mat drawMatches(cv::Mat img1, cv::Mat img2);

    cv::Mat drawGoodMatches(cv::Mat img1, cv::Mat img2);

    std::vector<double> getMatchCounts();

    void filter();

   private:
    int mKNN_K;

    std::shared_ptr<MasterObject> mObj;
    FrameData mReferenceFrameData;

    cv::Ptr<cv::FeatureDetector> mDetector;
    cv::Ptr<cv::DescriptorExtractor> mExtractor;
    cv::FlannBasedMatcher mMatcher;

    std::vector<cv::KeyPoint> mDummyKp;
    cv::Mat mDummyDes;

    std::vector<std::vector<cv::DMatch>> mDummyMatches;
    std::vector<cv::DMatch> mDummyGoodMatches;
};

#endif
