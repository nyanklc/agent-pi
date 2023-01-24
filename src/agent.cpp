#include "../include/agent.h"
#include "../include/globals.h"

Agent::Agent() {
  mTurnTolerance = TURN_TOLERANCE;

  mCameraCalibrator = CameraCalibrator();
  mFeatureMatcher = FeatureMatcher();

  mCalibrated = false;
}

bool Agent::process(cv::Mat &frame) {
  double obj_dist = 0;

  if (!mFeatureMatcher.findObject(frame)) {
    std::cout << "Agent couldn't find the object.";
    return false;
  }

  mObjData = mFeatureMatcher.getObjData();

  // TODO:

  return true;
}

