#include "../include/april_tag_detector.h"

AprilTagDetector::AprilTagDetector() {
}

void AprilTagDetector::init(std::shared_ptr<MasterObject> master_obj) {
  mFamily = tagStandard41h12_create(); 
  mDetector = apriltag_detector_create();

  // 2 bits is recommended by the creators, >=3 uses a lot of memory
  // But higher the better
  // apriltag_detector_add_family_bits(mDetector, mFamily, APRILTAG_FAMILY_BIT_COUNT);
  apriltag_detector_add_family(mDetector, mFamily);

  if (errno == ENOMEM) {
        printf("Unable to add family to detector due to insufficient memory to allocate the tag-family decoder with the default maximum hamming value of 2. Try choosing an alternative tag family.\n");
    }

  mDetector->quad_decimate = APRILTAG_QUAD_DECIMATE;
  mDetector->quad_sigma = APRILTAG_QUAD_SIGMA;
  mDetector->nthreads = APRILTAG_THREAD_COUNT;
  mDetector->refine_edges = APRILTAG_REFINE_EDGES;
  mDetector->nthreads = APRILTAG_THREAD_COUNT;
  if (APRILTAG_DEBUG_ON)
    mDetector->debug = true;
  else
    mDetector->debug = false;

  mObj = master_obj;
}

AprilTagDetector::~AprilTagDetector() {
  apriltag_detections_destroy(mDetections);
  apriltag_detector_destroy(mDetector);
  tagStandard41h12_destroy(mFamily);
}

bool AprilTagDetector::findObject(cv::Mat &frame) {
  // TODO: detector_detect spawns a thread every time,
  // this results in slower operation. Maybe modify apriltag source code
  // so that the required number of threads is kept throughout the operation.
  // TODO: maybe make im member so we don't allocate every time

  // if (mDetections != NULL)
  //   apriltag_detections_destroy(mDetections);

  // convert to apriltag image type
  image_u8_t im = {.width = frame.cols,
      .height = frame.rows,
      .stride = frame.cols,
      .buf = frame.data};

  // detect
  errno = 0; // stupid piece of shit
  mDetections = apriltag_detector_detect(mDetector, &im);

  if (errno == EAGAIN) {
    std::cout << "Unable to create the" << mDetector->nthreads << " threads requested.\n";
    return false;
  }

  // not detected
  if (zarray_size(mDetections) < 1)
    return false;

  return true;
}

zarray *AprilTagDetector::getDetections() {
  return mDetections;
}

void AprilTagDetector::drawDetections(cv::Mat &frame) {
  // Draw detection outlines
  for (int i = 0; i < zarray_size(mDetections); i++) {
      apriltag_detection_t *det;
      zarray_get(mDetections, i, &det);
      cv::Point p1(det->p[0][0], det->p[0][1]);
      cv::Point p2(det->p[1][0], det->p[1][1]);
      cv::Point p3(det->p[2][0], det->p[2][1]);
      cv::Point p4(det->p[3][0], det->p[3][1]);
      
      cv::line(frame, p1, p2, cv::Scalar(100, 100, 255), 10);
      cv::line(frame, p1, p4, cv::Scalar(100, 100, 255), 10);
      cv::line(frame, p2, p3, cv::Scalar(100, 100, 255), 10);
      cv::line(frame, p3, p4, cv::Scalar(100, 100, 255), 10);
  }
}





