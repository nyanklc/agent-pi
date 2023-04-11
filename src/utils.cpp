#include "../include/utils.h"

void printProjection(cv::Vec3f obj, cv::Vec2f img, std::string msg) {
    if (msg != "")
        std::cout << msg << ":\n";
    std::cout << "3D \t\t 2D\n";
    std::cout << "x: " << obj[0] << "\t\t"
              << "x: " << img[0] << "\n";
    std::cout << "y: " << obj[1] << "\t\t"
              << "y: " << img[1] << "\n";
    std::cout << "z: " << obj[2] << "\n";
}

void printProjection(cv::Point3f obj, cv::Point2f img, std::string msg) {
    if (msg != "")
        std::cout << msg << ":\n";
    std::cout << "3D \t\t 2D\n";
    std::cout << "x: " << obj.x << "\t\t"
              << "x: " << img.x << "\n";
    std::cout << "y: " << obj.y << "\t\t"
              << "y: " << img.y << "\n";
    std::cout << "z: " << obj.z << "\n";
}

void printMat(cv::Mat m, std::string msg) {
    if (msg != "")
        std::cout << msg << ":\n";
    for (int i = 0; i < m.rows; i++) {
        for (int j = 0; j < m.cols; j++) {
            std::cout << m.at<double>(i, j) << "\t";
        }
        std::cout << std::endl;
    }
}

void printVec3f(cv::Vec3f m, std::string msg) {
    if (msg != "")
        std::cout << msg << ":\n";
    std::cout << "0: " << m[0] << "\n";
    std::cout << "1: " << m[1] << "\n";
    std::cout << "2: " << m[2] << "\n";
}

void printPoint3f(cv::Point3f m, std::string msg) {
    if (msg != "")
        std::cout << msg << ":\n";
    std::cout << "x: " << m.x << "\n";
    std::cout << "y: " << m.y << "\n";
    std::cout << "z: " << m.z << "\n";
}

void printPoint2f(cv::Point2f m, std::string msg) {
    if (msg != "")
        std::cout << msg << ":\n";
    std::cout << "x: " << m.x << "\n";
    std::cout << "y: " << m.y << "\n";
}

// TODO: we're using zero distortion matrix right now
cv::Mat getDistortionMatrix() {
    // Create zero distortion
    cv::Mat distCoeffs(4, 1, cv::DataType<double>::type);
    distCoeffs.at<double>(0) = 0;
    distCoeffs.at<double>(1) = 0;
    distCoeffs.at<double>(2) = 0;
    distCoeffs.at<double>(3) = 0;
    return distCoeffs;
}

cv::Mat getCameraMatrix(double fx, double fy, double cx, double cy) {
    cv::Mat cam(3, 3, cv::DataType<double>::type);
    cam.at<double>(0, 0) = fx;
    cam.at<double>(0, 1) = 0;
    cam.at<double>(0, 2) = cx;
    cam.at<double>(1, 0) = 0;
    cam.at<double>(1, 1) = fy;
    cam.at<double>(1, 2) = cy;
    cam.at<double>(2, 0) = 0;
    cam.at<double>(2, 1) = 0;
    cam.at<double>(2, 2) = 1;
    return cam;
}

std::vector<cv::Vec3f> convertToVec3fVec(std::vector<cv::Point3f> pVec) {
    std::vector<cv::Vec3f> vec;
    for (int i = 0; i < pVec.size(); i++)
        vec.push_back(convertToVec3f(pVec[i]));
    return vec;
}

cv::Vec3f convertToVec3f(cv::Point3f &p) {
    cv::Vec3f v;
    for (int i = 0; i < 3; i++) {
        v[0] = p.x;
        v[1] = p.y;
        v[2] = p.z;
    }
    return v;
}

cv::Mat convertToMat(matd_t *m) {
    cv::Mat mat(m->nrows, m->ncols, cv::DataType<double>::type);
    for (int i = 0; i < m->nrows; i++)
        for (int j = 0; j < m->ncols; j++) {
            mat.at<double>(i, j) = matd_get(m, i, j);
        }
    return mat;
}

cv::Mat convertToMat(std::vector<cv::Point3f> &v) {
    cv::Mat m(3, v.size(), cv::DataType<double>::type);
    for (int i = 0; i < v.size(); i++) {
        m.at<double>(0, i) = v[i].x;
        m.at<double>(1, i) = v[i].y;
        m.at<double>(2, i) = v[i].z;
    }
    return m;
}

cv::Mat convertToMat(std::vector<cv::Point2f> &v) {
    cv::Mat m(2, v.size(), cv::DataType<double>::type);
    for (int i = 0; i < v.size(); i++) {
        m.at<double>(0, i) = v[i].x;
        m.at<double>(1, i) = v[i].y;
    }
    return m;
}

cv::Vec3f convertToVec3f(matd_t *m) {
    cv::Vec3f vec;
    for (int i = 0; i < 3; i++)
        vec[i] = m->data[i];
    return vec;
}

// defines a cube with side length = size * 2
std::vector<cv::Point3f> defineCubeWithPoints(double size) {
    std::vector<cv::Point3f> ret;
    ret.push_back(cv::Point3f(0, 0, 0));            // front bottom left
    ret.push_back(cv::Point3f(size, 0, 0));         // front bottom right
    ret.push_back(cv::Point3f(size, size, 0));      // front top right
    ret.push_back(cv::Point3f(0, size, 0));         // front top left
    ret.push_back(cv::Point3f(0, 0, -size));        // back bottom left
    ret.push_back(cv::Point3f(size, 0, -size));     // back bottom right
    ret.push_back(cv::Point3f(size, size, -size));  // back top right
    ret.push_back(cv::Point3f(0, size, -size));     // back top left
    // translate so that center is at the origin
    cv::Mat t(3, 1, cv::DataType<double>::type);
    t.at<double>(0, 0) = -size / 2;
    t.at<double>(1, 0) = -size / 2;
    t.at<double>(2, 0) = 0;
    translatePoints(ret, t);
    return ret;
}

// TODO: change values
std::vector<cv::Vec3f> defineCubeWithVectors(double side_length) {
    std::vector<cv::Vec3f> ret;

    cv::Vec3f corner;
    corner[0] = side_length / 2;
    corner[1] = -side_length / 2;
    corner[2] = side_length / 2;
    ret.push_back(corner);

    cv::Vec3f corner1;
    corner1[0] = side_length / 2;
    corner1[1] = side_length / 2;
    corner1[2] = side_length / 2;
    ret.push_back(corner1);

    cv::Vec3f corner2;
    corner2[0] = -side_length / 2;
    corner2[1] = side_length / 2;
    corner2[2] = side_length / 2;
    ret.push_back(corner2);

    cv::Vec3f corner3;
    corner3[0] = -side_length / 2;
    corner3[1] = -side_length / 2;
    corner3[2] = side_length / 2;
    ret.push_back(corner3);

    cv::Vec3f corner4;
    corner4[0] = side_length / 2;
    corner4[1] = -side_length / 2;
    corner4[2] = -side_length / 2;
    ret.push_back(corner4);

    cv::Vec3f corner5;
    corner5[0] = side_length / 2;
    corner5[1] = side_length / 2;
    corner5[2] = -side_length / 2;
    ret.push_back(corner5);

    cv::Vec3f corner6;
    corner6[0] = -side_length / 2;
    corner6[1] = side_length / 2;
    corner6[2] = -side_length / 2;
    ret.push_back(corner6);

    cv::Vec3f corner7;
    corner7[0] = -side_length / 2;
    corner7[1] = -side_length / 2;
    corner7[2] = -side_length / 2;
    ret.push_back(corner7);

    return ret;
}

void rotatePoints(std::vector<cv::Point3f> &cube, cv::Mat R) {
    for (auto &point : cube) {
        double tempx = point.x;
        double tempy = point.y;
        double tempz = point.z;
        point.x = R.at<double>(0, 0) * tempx + R.at<double>(0, 1) * tempy +
                  R.at<double>(0, 2) * tempz;
        point.y = R.at<double>(1, 0) * tempx + R.at<double>(1, 1) * tempy +
                  R.at<double>(1, 2) * tempz;
        point.z = R.at<double>(2, 0) * tempx + R.at<double>(2, 1) * tempy +
                  R.at<double>(2, 2) * tempz;
    }
}

// t is column vector
void translatePoints(std::vector<cv::Point3f> &cube, cv::Mat t) {
    for (auto &point : cube) {
        point.x += t.at<double>(0, 0);
        point.y += t.at<double>(1, 0);
        point.z += t.at<double>(2, 0);
    }
}

void drawCube(std::vector<cv::Point3f> &cube, cv::Mat &frame,
              cv::Mat &cameraMatrix, cv::Mat &distortionCoefficients,
              cv::Mat &rotationMatrix, cv::Mat &translationMatrix,
              cv::Scalar &color) {
    // Define a 3D transformation matrix that transforms coordinates from the
    // camera's coordinate system to the apriltag's coordinate system
    // Convert rotation and translation to 4x4 transformation matrix
    cv::Mat transformationMatrix = cv::Mat::eye(4, 4, cv::DataType<double>::type);
    cv::Mat submatrix = transformationMatrix(cv::Rect(0, 0, 3, 3));
    rotationMatrix.copyTo(submatrix);
    translationMatrix.copyTo(transformationMatrix(cv::Rect(3, 0, 1, 3)));
    // printMat(transformationMatrix, "transformationMatrix");

    // Transform the 3D coordinates of the cube vertices into the apriltag's
    // coordinate system
    std::vector<cv::Point3f> transformedCube;
    cv::perspectiveTransform(cube, transformedCube, transformationMatrix);

    // Project the transformed cube vertices onto the image plane
    std::vector<cv::Point2f> imagePoints;
    cv::projectPoints(transformedCube, cv::Mat::eye(3, 3, CV_64F),
                      cv::Mat::zeros(1, 3, CV_64F), cameraMatrix,
                      distortionCoefficients, imagePoints);

    // Draw the cube on the image
    cv::line(frame, imagePoints[0], imagePoints[1], color,
             1);  // front bottom left to front bottom right
    cv::line(frame, imagePoints[1], imagePoints[2], color,
             1);  // front bottom right to front top right
    cv::line(frame, imagePoints[2], imagePoints[3], color,
             1);  // front top right to front top left
    cv::line(frame, imagePoints[3], imagePoints[0], color,
             1);  // front top left to front bottom left
    cv::line(frame, imagePoints[4], imagePoints[5], color,
             1);  // back bottom left to back bottom right
    cv::line(frame, imagePoints[5], imagePoints[6], color,
             1);  // back bottom right to back top right
    cv::line(frame, imagePoints[6], imagePoints[7], color,
             1);  // back top right to back top left
    cv::line(frame, imagePoints[7], imagePoints[4], color,
             1);  // back top left to back bottom left
    cv::line(frame, imagePoints[0], imagePoints[4], color,
             1);  // front bottom left to back bottom left
    cv::line(frame, imagePoints[1], imagePoints[5], color,
             1);  // front bottom right to back bottom right
    cv::line(frame, imagePoints[2], imagePoints[6], color,
             1);  // front top right to back top right
    cv::line(frame, imagePoints[3], imagePoints[7], color,
             1);  // front top left to back top left
}

std::vector<cv::Point3f> defineAxesWithPoints(double size) {
    std::vector<cv::Point3f> pts;
    pts.push_back(cv::Point3f(0, 0, 0));     // origin
    pts.push_back(cv::Point3f(size, 0, 0));  // x
    pts.push_back(cv::Point3f(0, size, 0));  // y
    pts.push_back(cv::Point3f(0, 0, size));  // z
    return pts;
}

void drawAxes(std::vector<cv::Point3f> &axes, cv::Mat &frame,
              cv::Mat &cameraMatrix, cv::Mat &distortionCoefficients,
              cv::Mat &rotationMatrix, cv::Mat &translationMatrix,
              std::vector<cv::Scalar> &colors) {
    // Define a 3D transformation matrix that transforms coordinates from the
    // camera's coordinate system to the apriltag's coordinate system
    // Convert rotation and translation to 4x4 transformation matrix
    cv::Mat transformationMatrix = cv::Mat::eye(4, 4, cv::DataType<double>::type);
    cv::Mat submatrix = transformationMatrix(cv::Rect(0, 0, 3, 3));
    rotationMatrix.copyTo(submatrix);
    translationMatrix.copyTo(transformationMatrix(cv::Rect(3, 0, 1, 3)));
    // printMat(transformationMatrix, "transformationMatrix");

    // Transform the 3D coordinates of the cube vertices into the apriltag's
    // coordinate system
    std::vector<cv::Point3f> transformedCube;
    cv::perspectiveTransform(axes, transformedCube, transformationMatrix);

    // Project the transformed cube vertices onto the image plane
    std::vector<cv::Point2f> imagePoints;
    cv::projectPoints(transformedCube, cv::Mat::eye(3, 3, CV_64F),
                      cv::Mat::zeros(1, 3, CV_64F), cameraMatrix,
                      distortionCoefficients, imagePoints);

    // Draw the cube on the image
    cv::line(frame, imagePoints[0], imagePoints[1], colors[0],
             1);  // front bottom left to front bottom right
    cv::line(frame, imagePoints[0], imagePoints[2], colors[1],
             1);  // front bottom right to front top right
    cv::line(frame, imagePoints[0], imagePoints[3], colors[2],
             1);  // front top right to front top left
}

std::array<float, 3> getRPY(cv::Mat &R) {
    float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));
    bool singular = sy < 1e-6;  // If
    float x, y, z;
    if (!singular) {
        x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
        y = atan2(-R.at<double>(2, 0), sy);
        z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
    } else {
        x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
        y = atan2(-R.at<double>(2, 0), sy);
        z = 0;
    }
    std::array<float, 3> ret;
    ret[0] = x;
    ret[1] = y;
    ret[2] = z;
    return ret;
}

cv::Mat fromRPY(float roll, float pitch, float yaw) {
    // Calculate rotation about x axis
    cv::Mat R_x = (cv::Mat_<double>(3, 3) << 1, 0, 0,
                   0, cos(roll), -sin(roll),
                   0, sin(roll), cos(roll));
    // Calculate rotation about y axis
    cv::Mat R_y = (cv::Mat_<double>(3, 3) << cos(pitch), 0, sin(pitch),
                   0, 1, 0,
                   -sin(pitch), 0, cos(pitch));
    // Calculate rotation about z axis
    cv::Mat R_z = (cv::Mat_<double>(3, 3) << cos(yaw), -sin(yaw), 0,
                   sin(yaw), cos(yaw), 0,
                   0, 0, 1);
    // Combined rotation matrix
    cv::Mat R = R_z * R_y * R_x;
    return R;
}

Transform constructTransform(matd_t *R, matd_t *t) {
    return constructTransform(convertToMat(R), convertToMat(t));
}

Transform constructTransform(cv::Mat R, cv::Mat t) {
    cv::Mat transform_matrix = cv::Mat::eye(4, 4, CV_64FC1);
    R.copyTo(transform_matrix(cv::Rect(0, 0, 3, 3)));
    t.copyTo(transform_matrix(cv::Rect(3, 0, 1, 3)));

    Transform tf;
    tf.T = transform_matrix;
    return tf;
}

cv::Mat getRotationMatrix(double roll, double pitch, double yaw) {
    cv::Mat R = cv::Mat::zeros(3, 3, CV_64F);

    double c_roll = std::cos(roll);
    double s_roll = std::sin(roll);
    double c_pitch = std::cos(pitch);
    double s_pitch = std::sin(pitch);
    double c_yaw = std::cos(yaw);
    double s_yaw = std::sin(yaw);

    R.at<double>(0, 0) = c_yaw * c_pitch;
    R.at<double>(0, 1) = c_yaw * s_pitch * s_roll - s_yaw * c_roll;
    R.at<double>(0, 2) = c_yaw * s_pitch * c_roll + s_yaw * s_roll;

    R.at<double>(1, 0) = s_yaw * c_pitch;
    R.at<double>(1, 1) = s_yaw * s_pitch * s_roll + c_yaw * c_roll;
    R.at<double>(1, 2) = s_yaw * s_pitch * c_roll - c_yaw * s_roll;

    R.at<double>(2, 0) = -s_pitch;
    R.at<double>(2, 1) = c_pitch * s_roll;
    R.at<double>(2, 2) = c_pitch * c_roll;

    return R;
}

cv::Mat getRotationFromTransform(Transform &tf) {
    cv::Mat R = cv::Mat::zeros(3, 3, CV_64F);
    R.at<double>(0, 0) = tf.T.at<double>(0, 0);
    R.at<double>(0, 1) = tf.T.at<double>(0, 1);
    R.at<double>(0, 2) = tf.T.at<double>(0, 2);
    R.at<double>(1, 0) = tf.T.at<double>(1, 0);
    R.at<double>(1, 1) = tf.T.at<double>(1, 1);
    R.at<double>(1, 2) = tf.T.at<double>(1, 2);
    R.at<double>(2, 0) = tf.T.at<double>(2, 0);
    R.at<double>(2, 1) = tf.T.at<double>(2, 1);
    R.at<double>(2, 2) = tf.T.at<double>(2, 2);
    return R;
}

cv::Mat getTranslationFromTransform(Transform &tf) {
    cv::Mat t = cv::Mat::zeros(3, 1, CV_64F);
    t.at<double>(0, 0) = tf.T.at<double>(0, 3);
    t.at<double>(1, 0) = tf.T.at<double>(1, 3);
    t.at<double>(2, 0) = tf.T.at<double>(2, 3);
    return t;
}

void printTransform(Transform &tf, std::string msg) {
    if (msg != "")
        std::cout << msg << std::endl;
    auto R = getRotationFromTransform(tf);
    auto t = getTranslationFromTransform(tf);
    std::cout << "tf Rotation: " << R << std::endl;
    std::cout << "tf Translation: " << t << std::endl;
    auto rpy = getRPY(R);
    std::cout << "tf RPY: " << rpy[0] << ", " << rpy[1] << ", " << rpy[2] << std::endl;
}

cv::Mat getTranslationMatrix(double x, double y, double z) {
    cv::Mat t = cv::Mat::zeros(3, 1, CV_64F);
    t.at<double>(0, 0) = x;
    t.at<double>(1, 0) = y;
    t.at<double>(2, 0) = z;
    return t;
}

cv::Mat truncateVector(cv::Mat t, double fraction) {
    cv::Mat shorter = t;
    for (int i = 0; i < shorter.rows; i++)
        for (int j = 0; j < shorter.cols; j++)
            shorter.at<double>(i, j) = fraction * shorter.at<double>(i, j);
    return shorter;
}

double getAngularDifference(double th1, double th2) {
    using namespace std;

    if (th1 == th2)
        return 0.0;

    double diff = atan2((cos(th1) * sin(th2) - cos(th2) * sin(th1)),
                        (sin(th1) * sin(th2) + cos(th1) * cos(th2)));

    while (diff > M_PI)
        diff -= 2 * M_PI;
    while (diff < -M_PI)
        diff += 2 * M_PI;

    return diff;
}