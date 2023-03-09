#ifndef __AGENT_H
#define __AGENT_H

#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>

#include "april_tag_detector.h"
#include "camera_calibrator.h"
#include "controls.h"
#include "globals.h"
#include "master_object.h"

class Agent
{
public:
  Agent();

  std::vector<TagPose> process(cv::Mat &frame);

  void drawDetections(cv::Mat &frame, bool cube_on, bool axes_on);

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
  std::shared_ptr<AprilTagDetector> mApriltagDetector;
  double mTurnTolerance;
  std::shared_ptr<MasterObject> mObj;
};

#endif
