#include "../include/agent.h"

Agent::Agent()
{
  mTurnTolerance = TURN_TOLERANCE;

  mObj = std::make_shared<MasterObject>();

  mCameraCalibrator = CameraCalibrator();

  // When agent is created, it calls the default constructor of
  // mFeatureMatcher. I'm setting necessary stuff later on with init etc.
  mFeatureMatcher = FeatureMatcher();
  mFeatureMatcher.init(REFERENCE_IMG_PATH, KNN_K);
  mFeatureMatcher.setObj(mObj);

  mApriltagDetector = std::make_shared<AprilTagDetector>();
  mApriltagDetector->init(mObj);

  mCalibrated = true;
  setFocalLength((CAMERA_FX + CAMERA_FY) / 2);
}

void Agent::drawDetections(cv::Mat &frame)
{
  if (!GUI_ON)
    return;

  if (APRILTAG_ENABLED)
  {
    mApriltagDetector->drawDetections(frame);
    mApriltagDetector->drawMarkers(frame);
  }
  else if (KNN_ENABLED)
  {
    mFeatureMatcher.drawDetections(frame);
  }
}

void Agent::printDetections()
{
  std::vector<cv::Point> detectedPoints =
      mApriltagDetector->getDetectionPoints();
  for (size_t i = 0; i < detectedPoints.size(); i++)
    std::cout << "x" << i + 1 << ": " << detectedPoints[i].x << ", y" << i + 1
              << ": " << detectedPoints[i].y << "\n";
}

bool Agent::process(cv::Mat &frame)
{
  auto start = cv::getTickCount();
  if (KNN_ENABLED)
  {
    if (mFeatureMatcher.findObject(frame))
    {
      std::cout << "processing fps: "
                << cv::getTickFrequency() / (cv::getTickCount() - start)
                << "\n";
      return true;
    }
    std::cout << "processing fps: "
              << cv::getTickFrequency() / (cv::getTickCount() - start) << "\n";
    return false;
  }
  if (APRILTAG_ENABLED)
  {
    if (!mApriltagDetector->process(frame))
      return false;

    std::cout << "processing fps: "
              << cv::getTickFrequency() / (cv::getTickCount() - start) << "\n";
    return true;
  }
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
