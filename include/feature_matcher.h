#ifndef __FEATURE_MATCHER_H
#define __FEATURE_MATCHER_H

#include "master_object.h"

#include <memory>

#include <opencv2/opencv.hpp>

class FeatureMatcher {
  // TODO:
public:
  bool findObject(cv::Mat &frame);

  std::shared_ptr<MasterObject> getObjData();
};

#endif
