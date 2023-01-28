#include "../../include/april_tag_detector.h"

#include <opencv2/opencv.hpp>

#include <string>
#include <iostream>

const std::string img_path = "./april_tag_test.png";

int main(int argc, char **argv) {

  std::cout << "before atd\n";
  AprilTagDetector atd;
  std::cout << "after atd\n";

  cv::Mat frame = cv::imread(img_path);

  std::cout << "before find\n";
  atd.findObject(frame);
  std::cout << "after find\n";

  atd.drawDetections(frame);

  cv::imshow("april_tag_test", frame);
  cv::waitKey();
}
