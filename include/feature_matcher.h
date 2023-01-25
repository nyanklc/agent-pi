#ifndef __FEATURE_MATCHER_H
#define __FEATURE_MATCHER_H

#include "master_object.h"
#include "globals.h"

#include <memory>
#include <vector>

#include <opencv2/opencv.hpp>

struct FrameData {
  std::vector<cv::KeyPoint> kp;
  cv::Mat des;
};

class FeatureMatcher {
  // TODO:
public:
  FeatureMatcher();

  // virtual const.
  void init(std::string ref_img_path, int knn_k);

  void getReferenceFrame(std::string img_path);

  bool findObject(cv::Mat &frame);

  void match(cv::Mat &des, int k);

  void setObj(std::shared_ptr<MasterObject> m);

  cv::Mat drawMatches(cv::Mat img1, cv::Mat img2);

  cv::Mat drawGoodMatches(cv::Mat img1, cv::Mat img2);

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
