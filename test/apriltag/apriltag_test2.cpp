#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

#include "../../include/april_tag_detector.h"
#include "../../include/master_object.h"

const std::string img_path = "../test/apriltag/test3.jpg";

int main(int argc, char **argv) {
  std::cout << "#############################\n";

  std::cout << "creating master object\n";
  std::shared_ptr<MasterObject> mobj = std::make_shared<MasterObject>();
  std::cout << "created master object\n";

  std::cout << "#############################\n";

  std::cout << "creating detector\n";
  AprilTagDetector atd;
  atd.init(mobj);
  std::cout << "created detector\n";

  std::cout << "#############################\n";

  std::cout << "reading image\n";
  cv::Mat frame = cv::imread(img_path);
  cv::Mat frame_colored = frame;
  cv::cvtColor(frame, frame, cv::COLOR_BGRA2GRAY);

  std::cout << "#############################\n";

  std::cout << "detecting object\n";
  if (!atd.findObject(frame)) {
    std::cout << "detection failed\n";
  }
  printf("detected %d objects\n", zarray_size(atd.getDetections()));

  std::cout << "#############################\n";

  std::cout << "printing detections\n";
  auto detections = atd.getDetections();
  atd.printDetections(detections);

  std::cout << "#############################\n";

  std::cout << "estimating pose\n";
  if (!atd.poseEstimation(frame)) {
    std::cout << "pose estimation failed\n";
  }

  std::cout << "#############################\n";

  std::cout << "printing poses\n";
  auto poses = atd.getPoses();
  atd.printPoses(poses);

  std::cout << "#############################\n";

  std::cout << "drawing detections\n";
  atd.drawDetections(frame_colored);

  std::cout << "#############################\n";

  std::cout << "drawing markers\n";
  atd.drawMarkers(frame_colored);

  std::cout << "#############################\n";

  cv::imshow("result", frame_colored);
  cv::waitKey(0);
}
