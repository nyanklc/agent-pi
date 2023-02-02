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
        std::cout << "pose " << kk << ":\n";
        for (int i = 0; i < pose.R->nrows; i++) {
            for (int j = 0; j < pose.R->ncols; j++) {
                std::cout << matd_get(pose.R, i, j) << "\t";
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
        for (int i = 0; i < sizeof(det->p) / sizeof(det->p[0]); i++) {
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

        cv::line(frame, p1, p2, cv::Scalar(100, 100, 255), 10);
        cv::line(frame, p1, p4, cv::Scalar(100, 100, 255), 10);
        cv::line(frame, p2, p3, cv::Scalar(100, 100, 255), 10);
        cv::line(frame, p3, p4, cv::Scalar(100, 100, 255), 10);
        cv::circle(frame, c, 0.1, cv::Scalar(255, 0, 0));
    }
}

void AprilTagDetector::drawPoses(std::vector<apriltag_pose_t> &poses) {
    for (auto &pose : poses) {
        auto rot = pose.R;
        auto trans = pose.t;
    }
}
