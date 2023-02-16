#include "../include/april_tag_detector.h"

AprilTagDetector::AprilTagDetector() {}

void AprilTagDetector::init(std::shared_ptr<MasterObject> master_obj) {
  mFamily = tagStandard41h12_create();
  mDetector = apriltag_detector_create();

  // 2 bits is recommended by the creators, >=3 uses a lot of memory
  // But higher the better
  // apriltag_detector_add_family_bits(mDetector, mFamily,
  // APRILTAG_FAMILY_BIT_COUNT);
  apriltag_detector_add_family(mDetector, mFamily);

  if (errno == ENOMEM) {
    printf(
        "Unable to add family to detector due to insufficient memory to "
        "allocate the tag-family decoder with the default maximum hamming "
        "value of 2. Try choosing an alternative tag family.\n");
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

  mInfo.fx = CAMERA_FX;
  mInfo.fy = CAMERA_FY;
  mInfo.cx = CAMERA_CX;
  mInfo.cy = CAMERA_CY;
  mInfo.tagsize = APRILTAG_TAG_SIZE;

  // create axes (column vectors)
  mX = matd_create(3, 1);
  matd_put(mX, 0, 0, 1);
  mY = matd_create(3, 1);
  matd_put(mY, 1, 0, 1);
  mZ = matd_create(3, 1);
  matd_put(mZ, 2, 0, 1);
}

AprilTagDetector::~AprilTagDetector() {
  apriltag_detections_destroy(mDetections);
  apriltag_detector_destroy(mDetector);
  tagStandard41h12_destroy(mFamily);

  matd_destroy(mX);
  matd_destroy(mY);
  matd_destroy(mZ);

  // free memory of cube
  for (size_t m = mCube.size() - 1; m >= 0; m--) {
    matd_destroy(mCube[m]);
  }
}

bool AprilTagDetector::findObject(cv::Mat &frame) {
  // TODO: detector_detect spawns a thread every time,
  // this results in slower operation. Maybe modify apriltag source code
  // so that the required number of threads is kept throughout the operation.
  // TODO: maybe make im member so we don't allocate every time

  // convert to apriltag image type
  image_u8_t im = {.width = frame.cols,
                   .height = frame.rows,
                   .stride = frame.cols,
                   .buf = frame.data};

  // detect
  errno = 0;  // stupid piece of shit
  mDetections = apriltag_detector_detect(mDetector, &im);

  if (errno == EAGAIN) {
    std::cout << "Unable to create the" << mDetector->nthreads
              << " threads requested.\n";
    return false;
  }

  // not detected
  if (zarray_size(mDetections) < 1) return false;

  return true;
}

bool AprilTagDetector::poseEstimation(cv::Mat &frame) {
  bool success = true;

  std::vector<apriltag_pose_t> poses;
  for (int i = 0; i < zarray_size(mDetections); i++) {
    apriltag_detection_t *det;
    zarray_get(mDetections, i, &det);

    mInfo.det = det;

    apriltag_pose_t pose;
    double err = estimate_tag_pose(&mInfo, &pose);
    poses.push_back(pose);

    // std::cout << "err: " << err << "\n";
    if (err > APRILTAG_POSE_ERROR_THRESHOLD) success = false;
  }
  mPoses = poses;

  return success;
}

std::vector<apriltag_pose_t> AprilTagDetector::getPoses() { return mPoses; }

void AprilTagDetector::printPoses(std::vector<apriltag_pose_t> &poses) {
  int kk = 0;
  for (auto pose : poses) {
    kk++;
    std::cout << "pose " << kk << " rotation:\n";
    for (int i = 0; i < pose.R->nrows; i++) {
      for (int j = 0; j < pose.R->ncols; j++) {
        std::cout << matd_get(pose.R, i, j) << "\t";
      }
      std::cout << "\n";
    }
  }
  kk = 0;
  for (auto pose : poses) {
    kk++;
    std::cout << "pose " << kk << " tranlation:\n";
    for (int i = 0; i < pose.t->nrows; i++) {
      for (int j = 0; j < pose.t->ncols; j++) {
        std::cout << matd_get(pose.t, i, j) << "\t";
      }
      std::cout << "\n";
    }
  }
}

zarray *AprilTagDetector::getDetections() { return mDetections; }

std::vector<cv::Point> AprilTagDetector::getDetectionPoints() {
  std::vector<cv::Point> ret;

  for (int k = 0; k < zarray_size(mDetections); k++) {
    apriltag_detection_t *det;
    zarray_get(mDetections, k, &det);
    for (size_t i = 0; i < sizeof(det->p) / sizeof(det->p[0]); i++) {
      cv::Point p(det->p[i][0], det->p[i][1]);
      ret.push_back(p);
    }
  }

  return ret;
}

void AprilTagDetector::drawDetections(cv::Mat &frame) {
  // Draw detection outlines and midpoint
  for (int i = 0; i < zarray_size(mDetections); i++) {
    apriltag_detection_t *det;
    zarray_get(mDetections, i, &det);
    cv::circle(frame, cv::Point(det->c[0], det->c[1]), 1,
               cv::Scalar(255, 0, 0));
    cv::Point p1(det->p[0][0], det->p[0][1]);
    cv::Point p2(det->p[1][0], det->p[1][1]);
    cv::Point p3(det->p[2][0], det->p[2][1]);
    cv::Point p4(det->p[3][0], det->p[3][1]);

    cv::Point c(det->c[0], det->c[1]);

    cv::line(frame, p1, p2, cv::Scalar(100, 180, 0), 3);
    cv::line(frame, p1, p4, cv::Scalar(100, 180, 0), 3);
    cv::line(frame, p2, p3, cv::Scalar(100, 180, 0), 3);
    cv::line(frame, p3, p4, cv::Scalar(100, 180, 0), 3);
    cv::circle(frame, c, 2, cv::Scalar(0, 0, 255), 3);
  }
}

void AprilTagDetector::resetAxes() {
  auto tmpx = mX;
  auto tmpy = mY;
  auto tmpz = mZ;

  mX = matd_create(3, 1);
  matd_put(mX, 0, 0, 1);
  mY = matd_create(3, 1);
  matd_put(mY, 1, 0, 1);
  mZ = matd_create(3, 1);
  matd_put(mZ, 2, 0, 1);

  matd_destroy(tmpx);
  matd_destroy(tmpy);
  matd_destroy(tmpz);
}

void printMatv(std::vector<matd_t *> matv, int len) {
  for (int i = 0; i < len; i++) {
    std::cout << matv[i]->data[0] << "\t" << matv[i]->data[1] << "\t"
              << matv[i]->data[2] << "\n";
  }
}

void connectCorners(cv::Mat &frame, std::vector<matd_t *> matv) {
  // yes I could've written this in a loop, idc
  // 0 and 1
  cv::line(frame, cv::Point(matv[0]->data[0], matv[0]->data[1]),
           cv::Point(matv[2]->data[0], matv[2]->data[1]),
           cv::Scalar(255, 0, 0));
  cv::line(frame, cv::Point(matv[0]->data[0], matv[0]->data[1]),
           cv::Point(matv[3]->data[0], matv[3]->data[1]),
           cv::Scalar(255, 0, 0));
  cv::line(frame, cv::Point(matv[1]->data[0], matv[1]->data[1]),
           cv::Point(matv[2]->data[0], matv[2]->data[1]),
           cv::Scalar(255, 0, 0));
  cv::line(frame, cv::Point(matv[1]->data[0], matv[1]->data[1]),
           cv::Point(matv[3]->data[0], matv[3]->data[1]),
           cv::Scalar(255, 0, 0));
  // 4 and 5
  cv::line(frame, cv::Point(matv[4]->data[0], matv[4]->data[1]),
           cv::Point(matv[6]->data[0], matv[6]->data[1]),
           cv::Scalar(255, 0, 0));
  cv::line(frame, cv::Point(matv[4]->data[0], matv[4]->data[1]),
           cv::Point(matv[7]->data[0], matv[7]->data[1]),
           cv::Scalar(255, 0, 0));
  cv::line(frame, cv::Point(matv[5]->data[0], matv[5]->data[1]),
           cv::Point(matv[6]->data[0], matv[6]->data[1]),
           cv::Scalar(255, 0, 0));
  cv::line(frame, cv::Point(matv[5]->data[0], matv[5]->data[1]),
           cv::Point(matv[7]->data[0], matv[7]->data[1]),
           cv::Scalar(255, 0, 0));
  // plane connections
  cv::line(frame, cv::Point(matv[0]->data[0], matv[0]->data[1]),
           cv::Point(matv[4]->data[0], matv[4]->data[1]),
           cv::Scalar(255, 0, 0));
  cv::line(frame, cv::Point(matv[1]->data[0], matv[1]->data[1]),
           cv::Point(matv[5]->data[0], matv[5]->data[1]),
           cv::Scalar(255, 0, 0));
  cv::line(frame, cv::Point(matv[2]->data[0], matv[2]->data[1]),
           cv::Point(matv[6]->data[0], matv[6]->data[1]),
           cv::Scalar(255, 0, 0));
  cv::line(frame, cv::Point(matv[3]->data[0], matv[3]->data[1]),
           cv::Point(matv[7]->data[0], matv[7]->data[1]),
           cv::Scalar(255, 0, 0));
}

// remember to free memory after you call this
// note: destructor handles memory
std::vector<matd_t *> defineCube(double sideln) {
  double data[3];
  std::vector<matd_t *> matv;
  double side_len = sideln;  // px
  double y_APRILTAG_TAG_SIZE = -side_len;
  for (int i = 0; i < 8; i++) {
    auto mat = matd_create(3, 1);
    data[0] = (i % 2 == 0) ? side_len : -side_len;
    data[1] = y_APRILTAG_TAG_SIZE;
    if (i % 2 == 0) y_APRILTAG_TAG_SIZE *= -1;
    data[2] = (i < 4) ? 0 : side_len;

    matd_set_data(mat, data);
    matv.push_back(mat);
  }
  return matv;
}

void transformObject(std::vector<matd_t *> &matv, matd_t *R, matd_t *t,
                     apriltag_detection_t *det) {
  // transform cube
  for (size_t m = 0; m < matv.size(); m++) {
    auto temp = matd_add(matv[m], t);
    temp = matd_multiply(R, temp);

    // apply perspective
    // temp = matd_multiply(perspective_mat, temp);
    matv[m] = temp;
    // apriltag calculates rotation and translation weirdly.
    matv[m]->data[0] += det->c[0];  // to carry to the center of tag
    matv[m]->data[1] += det->c[1];
    matv[m]->data[2] *= -1;  // to draw the cube towards camera
  }
}

// TODO: return top view of the master
std::vector<cv::Point> AprilTagDetector::setMasterPosition() {}

double getTagSideLength(matd_t *R, matd_t *t, zarray_t *mDetections,
                        size_t &p) {
  // take two points of the tag's image points
  apriltag_detection_t *det;
  zarray_get(mDetections, p, &det);
  double doubles[3] = {0};
  double x1_x = det->p[1][0];
  double x1_y = det->p[1][1];
  double x2_x = det->p[2][0];
  double x2_y = det->p[2][1];

  doubles[0] = x1_x;
  doubles[1] = x1_y;
  matd_t *x1 = matd_create(3, 1);
  matd_set_data(x1, doubles);
  doubles[0] = x2_x;
  doubles[1] = x2_y;
  matd_t *x2 = matd_create(3, 1);
  matd_set_data(x2, doubles);
  // x1 and x2 are image points now

  matd_t *inv_R = matd_inverse(R);
  // x1 = matd_subtract(x1, t);
  // x2 = matd_subtract(x2, t);
  x1 = matd_multiply(inv_R, x1);
  x2 = matd_multiply(inv_R, x2);
  // x1 and x2 are object points now

  double side_len =
      std::hypot(x1->data[0] - x2->data[0], x1->data[1] - x2->data[1],
                 x1->data[2] - x2->data[2]);

  matd_destroy(inv_R);
  matd_destroy(x1);
  matd_destroy(x2);

  return side_len;
}

void AprilTagDetector::freeCube() {
  // free memory of cube
  for (size_t m = mCube.size() - 1; m >= 0; m--) {
    matd_destroy(mCube[m]);
  }
}

// ffs
void AprilTagDetector::drawCubes(cv::Mat &frame,
                                 std::vector<apriltag_pose_t> &poses) {
  for (size_t p = 0; p < poses.size(); p++) {
    // create cube
    // TODO: define side length equal to the side length of the tag
    apriltag_detection_t *det;
    zarray_get(mDetections, p, &det);
    // freeCube();
    mCube =
        defineCube(getTagSideLength(poses[p].R, poses[p].t, mDetections, p));

    // transform cube
    transformObject(mCube, poses[p].R, poses[p].t, det);

    // TODO:
    // project cube
    // double distance_x = CAMERA_FX;
    // double distance_y = distance_x;
    // for (int i = 0; i < matv.size(); i++) {
    //   matv[i]->data[0] *= distance_x / (distance_x +
    //   std::fabs(matv[i]->data[2])); matv[i]->data[1] *= distance_y /
    //   (distance_y + std::fabs(matv[i]->data[2]));
    // }

    // draw
    connectCorners(frame, mCube);
  }
}

std::vector<matd_t *> AprilTagDetector::getCube() { return mCube; }
