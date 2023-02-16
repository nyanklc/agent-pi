#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

#include "../../include/april_tag_detector.h"
#include "../../include/master_object.h"

const std::string img_path = "../test/apriltag/test2.png";

int main(int argc, char **argv) {
  std::cout << "creating master object\n";
  std::shared_ptr<MasterObject> mobj = std::make_shared<MasterObject>();
  std::cout << "created master object\n";

  std::cout << "creating detector\n";
  AprilTagDetector atd;
  atd.init(mobj);
  std::cout << "created detector\n";

  cv::VideoCapture cap(0);
  cv::Mat frame;

  while (1) {
    std::cout << "reading frame\n";
    cap.read(frame);
    cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);

    std::cout << "detecting object\n";
    if (!atd.findObject(frame)) {
      std::cout << "detection failed\n";
      continue;
    }
    printf("detected %d objects\n", zarray_size(atd.getDetections()));

    std::cout << "estimating pose\n";
    if (!atd.poseEstimation(frame)) {
      std::cout << "pose estimation failed\n";
      continue;
    }

    std::cout << "drawing detections\n";
    atd.drawDetections(frame);

    cv::imshow("april_tag_test", frame);
    cv::waitKey(1);
  }
}
