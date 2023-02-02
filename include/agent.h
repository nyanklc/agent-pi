#ifndef __AGENT_H
#define __AGENT_H

#include "globals.h"
#include "camera_calibrator.h"
#include "feature_matcher.h"
#include "master_object.h"
#include "controls.h"
#include "april_tag_detector.h"

#include <opencv2/opencv.hpp>

#include <memory>
#include <iostream>

class Agent
{
public:
  Agent();

  bool process(cv::Mat &frame);

  void drawDetections(cv::Mat &frame);

  void printDetections();

  Controls generateControls();

  double getFocalLength();

  void setFocalLength(double f);

  bool initFocalLengthCalibration(cv::Mat &frame);

  bool isCalibrated();

private:
  bool mCalibrated;
  double mFocalLength;
  CameraCalibrator mCameraCalibrator;
  FeatureMatcher mFeatureMatcher;
  std::shared_ptr<AprilTagDetector> mApriltagDetector;
  double mTurnTolerance;
  std::shared_ptr<MasterObject> mObj;

  FeatureMatcher *mFMPointer;
};

#endif
