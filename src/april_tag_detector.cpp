#include "../include/april_tag_detector.h"

#include <apriltag/apriltag.h>
#include <apriltag/common/matd.h>
#include <emmintrin.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

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

void AprilTagDetector::drawAxes(cv::Mat &frame,
                                std::vector<apriltag_pose_t> &poses) {
    // doesn't work lol

    size_t i = 0;
    for (auto &pose : poses) {
        auto rot = pose.R;
        auto trans = pose.t;

        // TODO:
        // i don't think this should be necessary
        // could be written more efficiently
        resetAxes();

        // rotate axes
        auto rotated_x = matd_multiply(rot, mX);
        auto rotated_y = matd_multiply(rot, mY);
        auto rotated_z = matd_multiply(rot, mZ);
        // translate axes
        auto x = matd_add(trans, mX);
        auto y = matd_add(trans, mY);
        auto z = matd_add(trans, mZ);

        // test
        apriltag_detection_t *det;
        zarray_get(mDetections, i, &det);
        double f = (CAMERA_FX + CAMERA_FY) / 2;
        auto x_x = f * matd_get(x, 0, 0) / matd_get(trans, 2, 0) / 100;
        std::cout << "x_x: " << x_x << std::endl;
        auto x_y = f * matd_get(x, 1, 0) / matd_get(trans, 2, 0) / 100;
        std::cout << "x_y: " << x_y << std::endl;
        auto y_x = f * matd_get(y, 0, 0) / matd_get(trans, 2, 0) / 100;
        std::cout << "y_x: " << y_x << std::endl;
        auto y_y = f * matd_get(y, 1, 0) / matd_get(trans, 2, 0) / 100;
        std::cout << "y_y: " << y_y << std::endl;
        auto z_x = f * matd_get(z, 0, 0) / matd_get(trans, 2, 0) / 100;
        std::cout << "z_x: " << z_x << std::endl;
        auto z_y = f * matd_get(z, 1, 0) / matd_get(trans, 2, 0) / 100;
        std::cout << "z_y: " << z_y << std::endl;
        cv::line(frame, cv::Point(det->c[0], det->c[1]), cv::Point(x_x, x_y),
                 cv::Scalar(0, 0, 255));  // x
        cv::line(frame, cv::Point(det->c[0], det->c[1]), cv::Point(y_x, y_y),
                 cv::Scalar(0, 255, 0));  // y
        cv::line(frame, cv::Point(det->c[0], det->c[1]), cv::Point(z_x, z_y),
                 cv::Scalar(255, 0, 0));  // z

        // draw
        // TODO:

        // cv::Mat objPoints();
        // cv::projectPoints();

        // free
        matd_destroy(rotated_x);
        matd_destroy(rotated_y);
        matd_destroy(rotated_z);

        i++;
    }
}

void printMatv(std::vector<matd_t *> matv, int len) {
    for (int i = 0; i < len; i++) {
        std::cout << matv[i]->data[0] << "\t" << matv[i]->data[1] << "\t"
                  << matv[i]->data[2] << "\n";
    }
}

// ffs
void AprilTagDetector::drawCubes(cv::Mat &frame,
                                 std::vector<apriltag_pose_t> &poses) {
    for (size_t p = 0; p < poses.size(); p++) {
        // create cube
        apriltag_detection_t *det;
        zarray_get(mDetections, p, &det);

        std::vector<matd_t *> matv;
        double data[3];

        double side_len = 20; // px
        double y_APRILTAG_TAG_SIZE = -side_len;        
        for (int i = 0; i < 8; i++) {
            auto mat = matd_create(3, 1);
            data[0] =
                (i % 2 == 0) ? side_len : -side_len;
            data[1] = y_APRILTAG_TAG_SIZE / 2;
            if (i % 2 == 0) y_APRILTAG_TAG_SIZE *= -1;
            data[2] = (i < 4) ? 0 : side_len;

            matd_set_data(mat, data);
            matv.push_back(mat);
        }

        std::cout << "before\n";
        printMatv(matv, 8);

        // transform cube
        for (size_t m = 0; m < 8; m++) {
            auto temp = matd_multiply(poses[p].R, matv[m]);
            temp = matd_add(temp, poses[p].t);
            matv[m] = temp;
            matv[m]->data[0] += det->c[0]; // to carry to the center of tag
            matv[m]->data[1] += det->c[1];
            matv[m]->data[2] *= -1; // to draw the cube towards camera
        }

        std::cout << "after\n";
        printMatv(matv, 8);

        // test (orthogonal projection)
        for (size_t i = 0; i < matv.size(); i++) {
            cv::circle(frame, cv::Point(matv[i]->data[0], matv[i]->data[1]), 3, cv::Scalar(255, 0, 0));
        }

        // free
        for (size_t m = 0; m < 8; m++) {
            matd_destroy(matv[m]);
        }
    }
}
