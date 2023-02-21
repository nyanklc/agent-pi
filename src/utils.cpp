#include "../include/utils.h"

void printProjection(cv::Vec3f obj, cv::Vec2f img, std::string msg) {
  if (msg != "") std::cout << msg << ":\n";
  std::cout << "3D \t\t 2D\n";
  std::cout << "x: " << obj[0] << "\t\t"
            << "x: " << img[0] << "\n";
  std::cout << "y: " << obj[1] << "\t\t"
            << "y: " << img[1] << "\n";
  std::cout << "z: " << obj[2] << "\n";
}

void printProjection(cv::Point3f obj, cv::Point2f img, std::string msg) {
  if (msg != "") std::cout << msg << ":\n";
  std::cout << "3D \t\t 2D\n";
  std::cout << "x: " << obj.x << "\t\t"
            << "x: " << img.x << "\n";
  std::cout << "y: " << obj.y << "\t\t"
            << "y: " << img.y << "\n";
  std::cout << "z: " << obj.z << "\n";
}

void printMat(cv::Mat m, std::string msg) {
  if (msg != "") std::cout << msg << ":\n";
  for (int i = 0; i < m.rows; i++) {
    for (int j = 0; j < m.cols; j++) {
      std::cout << m.at<double>(i, j) << "\t";
    }
    std::cout << std::endl;
  }
}

void printVec3f(cv::Vec3f m, std::string msg) {
  if (msg != "") std::cout << msg << ":\n";
  std::cout << "0: " << m[0] << "\n";
  std::cout << "1: " << m[1] << "\n";
  std::cout << "2: " << m[2] << "\n";
}

void printPoint3f(cv::Point3f m, std::string msg) {
  if (msg != "") std::cout << msg << ":\n";
  std::cout << "x: " << m.x << "\n";
  std::cout << "y: " << m.y << "\n";
  std::cout << "z: " << m.z << "\n";
}

void printPoint2f(cv::Point2f m, std::string msg) {
  if (msg != "") std::cout << msg << ":\n";
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
  for (int i = 0; i < pVec.size(); i++) vec.push_back(convertToVec3f(pVec[i]));
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
  for (int i = 0; i < 3; i++) vec[i] = m->data[i];
  return vec;
}

// defines a cube with side length = size * 2
std::vector<cv::Point3f> defineCubeWithPoints(double size) {
  std::vector<cv::Point3f> ret;
  ret.push_back(cv::Point3f(0, 0, 0));   // front bottom left
  ret.push_back(cv::Point3f(size, 0, 0));   // front bottom right
  ret.push_back(cv::Point3f(size, size, 0));   // front top right
  ret.push_back(cv::Point3f(0, size, 0));   // front top left
  ret.push_back(cv::Point3f(0, 0, -size));  // back bottom left
  ret.push_back(cv::Point3f(size, 0, -size));  // back bottom right
  ret.push_back(cv::Point3f(size, size, -size));  // back top right
  ret.push_back(cv::Point3f(0, size, -size));  // back top left
  // translate so that center is at the origin
  cv::Mat t(3, 1, cv::DataType<double>::type);
  t.at<double>(0, 0) = -size/2;
  t.at<double>(1, 0) = -size/2;
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
              cv::Mat &rotationMatrix, cv::Mat &translationMatrix) {
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
  cv::line(frame, imagePoints[0], imagePoints[1], cv::Scalar(0, 255, 0),
           2);  // front bottom left to front bottom right
  cv::line(frame, imagePoints[1], imagePoints[2], cv::Scalar(0, 255, 0),
           2);  // front bottom right to front top right
  cv::line(frame, imagePoints[2], imagePoints[3], cv::Scalar(0, 255, 0),
           2);  // front top right to front top left
  cv::line(frame, imagePoints[3], imagePoints[0], cv::Scalar(0, 255, 0),
           2);  // front top left to front bottom left
  cv::line(frame, imagePoints[4], imagePoints[5], cv::Scalar(0, 255, 0),
           2);  // back bottom left to back bottom right
  cv::line(frame, imagePoints[5], imagePoints[6], cv::Scalar(0, 255, 0),
           2);  // back bottom right to back top right
  cv::line(frame, imagePoints[6], imagePoints[7], cv::Scalar(0, 255, 0),
           2);  // back top right to back top left
  cv::line(frame, imagePoints[7], imagePoints[4], cv::Scalar(0, 255, 0),
           2);  // back top left to back bottom left
  cv::line(frame, imagePoints[0], imagePoints[4], cv::Scalar(0, 255, 0),
           2);  // front bottom left to back bottom left
  cv::line(frame, imagePoints[1], imagePoints[5], cv::Scalar(0, 255, 0),
           2);  // front bottom right to back bottom right
  cv::line(frame, imagePoints[2], imagePoints[6], cv::Scalar(0, 255, 0),
           2);  // front top right to back top right
  cv::line(frame, imagePoints[3], imagePoints[7], cv::Scalar(0, 255, 0),
           2);  // front top left to back top left
}