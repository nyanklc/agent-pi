#include "globals.h"

#include <opencv2/opencv.hpp>

// TODO: implement

class Agent {
public:
  Agent();

  void process();

  std::vector<char> generateControls();

  double getFocalLength();

  void setFocalLength();

  void initFocalLengthCalibration(cv::Mat frame);

  bool isCalibrated();

private:
  // TODO:
};
