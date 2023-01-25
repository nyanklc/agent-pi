#include "../include/agent.h"

Agent::Agent() {
  mTurnTolerance = TURN_TOLERANCE;

  mObj = std::make_shared<MasterObject>();

  mCameraCalibrator = CameraCalibrator();

  // When agent is created, it calls the default constructor of mFeatureMatcher,
  // in order to "give" it the same object, I'm just setting it after creation.
  mFeatureMatcher = FeatureMatcher();
  mFeatureMatcher.setObj(mObj);

  mCalibrated = false;
}

Agent::~Agent() {
  delete mFMPointer;
}

bool Agent::process(cv::Mat &frame) {
  // TODO: Maybe just used the shared master object,
  // and don't even check whether an object was detected in
  // the last frame. Use the latest data ??  
  // And make sure mObj is not accessed by detector and agent at
  // the same time. Maybe use a mutex and lock.
  // This is already a single thread process, and there is no need for
  // that right now. But we may want to take the feature matching to a
  // separate process.


  double obj_dist = 0;

  if (!mFeatureMatcher.findObject(frame)) {
    std::cout << "Agent couldn't find the object.";
    return false;
  }

  // TODO:

  return true;
}

Controls Agent::generateControls() {
  Controls controls;
  // TODO:
  return controls;
}

double Agent::getFocalLength() {
  return mFocalLength;
}

void Agent::setFocalLength(double f) {
  mFocalLength = f; 
}

bool Agent::initFocalLengthCalibration(cv::Mat &frame) {
  double measured_distance;
  double real_width;
  double width_in_rf_image;

  std::cout << "starting focal length calibration\n";
  std::cout << "make sure the object faces the camera los perpendicularly\n";
  std::cout << "input distance to object: ";
  std::cin >> measured_distance;
  std::cout << "input real width of the object (long line): ";
  std::cin >> real_width;

  if (!mFeatureMatcher.findObject(frame)) {
    std::cout << "object not detected\n";
    return false;
  }
  width_in_rf_image = mObj->width;
  
  mFocalLength = mCameraCalibrator.calculateFocalLength(measured_distance, real_width, width_in_rf_image);
  // TODO: get complete camera parameters

  if (mFocalLength == -1)
    return false;

  std::cout << "focal length calculated: " << mFocalLength << "\n";
  mCalibrated = true;
  return true;
}
