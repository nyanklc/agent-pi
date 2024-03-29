#include "../include/april_tag_detector.h"

AprilTagDetector::AprilTagDetector() {}

void AprilTagDetector::init()
{
    mFamily = tagStandard41h12_create();
    mDetector = apriltag_detector_create();

    // 2 bits is recommended by the creators, >=3 uses a lot of memory
    // But higher the better
    // apriltag_detector_add_family_bits(mDetector, mFamily,
    // APRILTAG_FAMILY_BIT_COUNT);
    apriltag_detector_add_family(mDetector, mFamily);

    if (errno == ENOMEM)
    {
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

    mInfo.fx = CAMERA_FX;
    mInfo.fy = CAMERA_FY;
    mInfo.cx = CAMERA_CX;
    mInfo.cy = CAMERA_CY;
    mInfo.tagsize = APRILTAG_TAG_SIZE;
}

AprilTagDetector::~AprilTagDetector()
{
    apriltag_detections_destroy(mDetections);
    apriltag_detector_destroy(mDetector);
    tagStandard41h12_destroy(mFamily);
}

std::vector<TagPose> AprilTagDetector::process(cv::Mat &frame)
{
    std::vector<TagPose> tag_poses;
    if (!findObject(frame))
    {
        // std::cout << "couldn't detect object\n";
        return tag_poses;
    }
    if (!poseEstimation(frame))
    {
        std::cout << "couldn't estimate pose\n";
        return tag_poses;
    }

    for (int i = 0; i < zarray_size(mDetections); i++)
    {
        apriltag_detection_t *det;
        zarray_get(mDetections, i, &det);

        TagPose pose;
        pose.id = det->id;
        pose.x = mPoses[i].t->data[0];
        pose.y = mPoses[i].t->data[1];
        pose.z = mPoses[i].t->data[2];
        auto rot = convertToMat(mPoses[i].R);
        std::array<float, 3> rpy = getRPY(rot);
        pose.roll = rpy[0];
        pose.pitch = rpy[1];
        pose.yaw = rpy[2];
        pose.center_x_px = det->c[0];
        pose.center_y_px = det->c[1];
        tag_poses.push_back(pose);
    }

    // // debug
    // for (int i = 0; i < tag_poses.size(); i++)
    // {
    //   std::cout << "pose " << i << " id: " << tag_poses[i].id << "\n";
    //   std::cout << "pose " << i << " x: " << tag_poses[i].x << "\n";
    //   std::cout << "pose " << i << " y: " << tag_poses[i].y << "\n";
    //   std::cout << "pose " << i << " z: " << tag_poses[i].z << "\n";
    //   std::cout << "pose " << i << " roll: " << tag_poses[i].roll << "\n";
    //   std::cout << "pose " << i << " pitch: " << tag_poses[i].pitch << "\n";
    //   std::cout << "pose " << i << " yaw: " << tag_poses[i].yaw << "\n";
    // }

    return tag_poses;
}

bool AprilTagDetector::findObject(cv::Mat &frame)
{
    // convert to apriltag image type
    image_u8_t im = {.width = frame.cols,
                     .height = frame.rows,
                     .stride = frame.cols,
                     .buf = frame.data};

    // detect
    errno = 0; // stupid piece of shit
    mDetections = apriltag_detector_detect(mDetector, &im);

    if (errno == EAGAIN)
    {
        std::cout << "Unable to create the" << mDetector->nthreads
                  << " threads requested.\n";
        return false;
    }

    // not detected
    if (zarray_size(mDetections) < 1)
        return false;

    return true;
}

bool AprilTagDetector::poseEstimation(cv::Mat &frame)
{
    bool success = true;

    std::vector<apriltag_pose_t> poses;
    for (int i = 0; i < zarray_size(mDetections); i++)
    {
        apriltag_detection_t *det;
        zarray_get(mDetections, i, &det);

        mInfo.det = det;

        apriltag_pose_t pose;
        double err = estimate_tag_pose(&mInfo, &pose);
        poses.push_back(pose);

        // test
        apriltag_pose_t pose1;
        apriltag_pose_t pose2;
        double err1;
        double err2;
        estimate_tag_pose_orthogonal_iteration(&mInfo, &err1, &pose1, &err2, &pose2, 3);

        // sometimes orthogonal iteration doesn't return 2 solutions
        if (pose2.R)
        {
            mPosesOrthogonal.clear();
            mPosesOrthogonal.push_back(std::pair<apriltag_pose_t, apriltag_pose_t>(pose1, pose2));
        }

        // std::cout << "err: " << err << "\n";
        if (err > APRILTAG_POSE_ERROR_THRESHOLD)
            success = false;
    }
    mPoses.clear();
    mPoses = poses;

    return success;
}

std::vector<apriltag_pose_t> AprilTagDetector::getPoses() { return mPoses; }

void AprilTagDetector::printPoses(std::vector<apriltag_pose_t> &poses)
{
    int kk = 0;
    for (auto pose : poses)
    {
        kk++;
        std::cout << "pose " << kk << " rotation:\n";
        for (int i = 0; i < pose.R->nrows; i++)
        {
            for (int j = 0; j < pose.R->ncols; j++)
            {
                std::cout << matd_get(pose.R, i, j) << "\t";
            }
            std::cout << "\n";
        }
    }
    kk = 0;
    for (auto pose : poses)
    {
        kk++;
        std::cout << "pose " << kk << " translation:\n";
        for (int i = 0; i < pose.t->nrows; i++)
        {
            for (int j = 0; j < pose.t->ncols; j++)
            {
                std::cout << matd_get(pose.t, i, j) << "\t";
            }
            std::cout << "\n";
        }
    }
}

zarray *AprilTagDetector::getDetections() { return mDetections; }

void AprilTagDetector::printDetections(zarray *detections)
{
    for (int k = 0; k < zarray_size(detections); k++)
    {
        apriltag_detection_t *det;
        zarray_get(detections, k, &det);
        std::cout << "detection " << k << ": \n";
        std::cout << "center x: " << det->c[0] << ", center y: " << det->c[1]
                  << std::endl;
        for (size_t i = 0; i < sizeof(det->p) / sizeof(det->p[0]); i++)
        {
            std::cout << "x: " << det->p[i][0] << ", y: " << det->p[i][1]
                      << std::endl;
        }
    }
}

std::vector<cv::Point> AprilTagDetector::getDetectionPoints()
{
    std::vector<cv::Point> ret;

    for (int k = 0; k < zarray_size(mDetections); k++)
    {
        apriltag_detection_t *det;
        zarray_get(mDetections, k, &det);
        for (size_t i = 0; i < sizeof(det->p) / sizeof(det->p[0]); i++)
        {
            cv::Point p(det->p[i][0], det->p[i][1]);
            ret.push_back(p);
        }
    }

    return ret;
}

void AprilTagDetector::drawDetections(cv::Mat &frame)
{
    // Draw detection outlines and midpoint
    for (int i = 0; i < zarray_size(mDetections); i++)
    {
        apriltag_detection_t *det;
        zarray_get(mDetections, i, &det);
        cv::circle(frame, cv::Point(det->c[0], det->c[1]), 1,
                   cv::Scalar(255, 0, 0));
        cv::Point p1(det->p[0][0], det->p[0][1]);
        cv::Point p2(det->p[1][0], det->p[1][1]);
        cv::Point p3(det->p[2][0], det->p[2][1]);
        cv::Point p4(det->p[3][0], det->p[3][1]);

        // cv::Point c(det->c[0], det->c[1]);

        cv::line(frame, p1, p2, cv::Scalar(100, 180, 0), 3);
        cv::line(frame, p1, p4, cv::Scalar(100, 180, 0), 3);
        cv::line(frame, p2, p3, cv::Scalar(100, 180, 0), 3);
        cv::line(frame, p3, p4, cv::Scalar(100, 180, 0), 3);
        // cv::circle(frame, c, 2, cv::Scalar(0, 0, 255), 3);
    }
}

void AprilTagDetector::drawMarkers(cv::Mat &frame, bool cube_on, bool axes_on)
{
    for (size_t k = 0; k < mPoses.size(); k++)
    {
        cv::Mat R = convertToMat(mPoses[k].R);
        // cv::Rodrigues(R, R); // convert to rotation vector
        // printMat(R, "R");

        cv::Mat t = convertToMat(mPoses[k].t);
        // printMat(t, "t");

        cv::Mat K = getCameraMatrix(CAMERA_FX, CAMERA_FY, CAMERA_CX, CAMERA_CY);
        // printMat(K, "K");

        cv::Mat distortion = getDistortionMatrix();
        // printMat(distortion, "distortion");

        if (cube_on)
        {
            // cubes
            auto cube = defineCubeWithPoints();
            auto color = cv::Scalar(0, 255, 0);
            drawCube(cube, frame, K, distortion, R, t, color);

            // // also draw second result from orthogonal iteration
            // // sometimes orthogonal iteration doesn't return 2 solutions
            // if (mPosesOrthogonal.size() > k && mPosesOrthogonal[k].second.R)
            // {
            //   // test
            //   std::cout << mPoses.size() << " " << mPosesOrthogonal.size() << std::endl;
            //   cv::Mat R1 = convertToMat(mPosesOrthogonal[k].first.R);
            //   // printMat(R1, "R1");
            //   cv::Mat R2 = convertToMat(mPosesOrthogonal[k].second.R);
            //   // printMat(R2, "R2");
            //   cv::Mat t1 = convertToMat(mPosesOrthogonal[k].first.t);
            //   // printMat(t1, "t1");
            //   cv::Mat t2 = convertToMat(mPosesOrthogonal[k].second.t);
            //   // printMat(t2, "t2");
            //   // auto color1 = cv::Scalar(255, 0, 0);
            //   // drawCube(cube, frame, K, distortion, R1, t1, color1);
            //   auto color2 = cv::Scalar(0, 0, 255);
            //   drawCube(cube, frame, K, distortion, R2, t2, color2);
            // }
        }

        if (axes_on)
        {
            // axes
            auto axes = defineAxesWithPoints();
            std::vector<cv::Scalar> axes_colors;
            axes_colors.push_back(cv::Scalar(0, 0, 255)); // x is red
            axes_colors.push_back(cv::Scalar(0, 255, 0)); // y is green
            axes_colors.push_back(cv::Scalar(255, 0, 0)); // z is blue
            drawAxes(axes, frame, K, distortion, R, t, axes_colors);
        }
    }
}
