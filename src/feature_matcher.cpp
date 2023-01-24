#include "../include/feature_matcher.h"

FeatureMatcher::FeatureMatcher() {
}

bool FeatureMatcher::findObject(cv::Mat &frame) {
    // TODO:
    return true;
}

void FeatureMatcher::setObj(std::shared_ptr<MasterObject> m) {
  mObj = m;
}
