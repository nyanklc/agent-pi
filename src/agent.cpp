#include "../include/agent.h"

Agent::Agent()
{
  mObj = std::make_shared<MasterObject>();

  mCameraCalibrator = CameraCalibrator();

  mApriltagDetector = std::make_shared<AprilTagDetector>();
  mApriltagDetector->init(mObj);

  mCalibrated = true;
  setFocalLength((CAMERA_FX + CAMERA_FY) / 2);
}

void Agent::drawDetections(cv::Mat &frame, bool cube_on, bool axes_on)
{
  if (!GUI_ON)
    return;

  mApriltagDetector->drawDetections(frame);
  mApriltagDetector->drawMarkers(frame, cube_on, axes_on);
}

void Agent::printDetections()
{
  std::vector<cv::Point> detectedPoints =
      mApriltagDetector->getDetectionPoints();
  for (size_t i = 0; i < detectedPoints.size(); i++)
    std::cout << "x" << i + 1 << ": " << detectedPoints[i].x << ", y" << i + 1
              << ": " << detectedPoints[i].y << "\n";
}

std::vector<TagPose> Agent::process(cv::Mat &frame)
{
  return mApriltagDetector->process(frame);
}

Controls Agent::generateControls()
{
  Controls controls;
  // TODO:
  return controls;
}

double Agent::getFocalLength() { return mFocalLength; }

void Agent::setFocalLength(double f) { mFocalLength = f; }

bool Agent::isCalibrated() { return mCalibrated; }
