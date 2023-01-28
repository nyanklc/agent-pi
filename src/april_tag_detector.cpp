#include "../include/april_tag_detector.h"
#include <apriltag/apriltag.h>

AprilTagDetector::AprilTagDetector() {
  mFamily = tagStandard41h12_create(); 
  mDetector = apriltag_detector_create();

  // 2 bits is recommended by the creators, >=3 uses a lot of memory
  // But higher the better
  apriltag_detector_add_family_bits(mDetector, mFamily, APRILTAG_FAMILY_BIT_COUNT);
  // apriltag_detector_add_family(mDetector, mFamily);

  // set detector parameters
  mDetector->nthreads = APRILTAG_THREAD_COUNT;
}

AprilTagDetector::~AprilTagDetector() {
  apriltag_detections_destroy(mDetections);
  apriltag_detector_destroy(mDetector);
  tagStandard41h12_destroy(mFamily);
}

bool AprilTagDetector::findObject(cv::Mat &frame) {
  // TODO: maybe make im member so we don't allocate every time
  // convert to apriltag image type
  image_u8_t im = {.width = frame.cols,
      .height = frame.rows,
      .stride = frame.cols,
      .buf = frame.data};

  // detect
  mDetections = apriltag_detector_detect(mDetector, &im);

  if (zarray_size(mDetections) < 1)
    return false;

  return true;
}

void AprilTagDetector::drawDetections(cv::Mat &frame) {
  // Draw detection outlines
  for (int i = 0; i < zarray_size(mDetections); i++) {
      apriltag_detection_t *det;
      zarray_get(mDetections, i, &det);
      line(frame, cv::Point(det->p[0][0], det->p[0][1]),
               cv::Point(det->p[1][0], det->p[1][1]),
               cv::Scalar(0, 0xff, 0), 2);
      line(frame, cv::Point(det->p[0][0], det->p[0][1]),
               cv::Point(det->p[3][0], det->p[3][1]),
               cv::Scalar(0, 0, 0xff), 2);
      line(frame, cv::Point(det->p[1][0], det->p[1][1]),
               cv::Point(det->p[2][0], det->p[2][1]),
               cv::Scalar(0xff, 0, 0), 2);
      line(frame, cv::Point(det->p[2][0], det->p[2][1]),
               cv::Point(det->p[3][0], det->p[3][1]),
               cv::Scalar(0xff, 0, 0), 2);
  }
}





