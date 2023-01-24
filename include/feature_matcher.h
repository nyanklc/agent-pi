#ifndef __FEATURE_MATCHER_H
#define __FEATURE_MATCHER_H

#include "master_object.h"

#include <memory>

#include <opencv2/opencv.hpp>

class FeatureMatcher {
  // TODO:
public:
  FeatureMatcher();

  bool findObject(cv::Mat &frame);

  void setObj(std::shared_ptr<MasterObject> m);

private:
  std::shared_ptr<MasterObject> mObj;
};

#endif
