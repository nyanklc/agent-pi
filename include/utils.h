#ifndef __UTILS_H
#define __UTILS_H

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

extern "C" {
#include <apriltag/apriltag.h>
#include <apriltag/apriltag_math.h>
#include <apriltag/apriltag_pose.h>
#include <apriltag/common/getopt.h>
#include <apriltag/common/matd.h>
#include <apriltag/tagStandard41h12.h>
}

void rotatePoints(std::vector<cv::Point3f> &cube, cv::Mat R);

void translatePoints(std::vector<cv::Point3f> &cube, cv::Mat t);

void printProjection(cv::Vec3f obj, cv::Vec2f img, std::string msg = "");

void printProjection(cv::Point3f obj, cv::Point2f img, std::string msg = "");

void printMat(cv::Mat m, std::string msg = "");

void printVec3f(cv::Vec3f m, std::string msg = "");

void printPoint3f(cv::Point3f m, std::string msg = "");

void printPoint2f(cv::Point2f m, std::string msg = "");

cv::Mat getDistortionMatrix();

cv::Mat getCameraMatrix(double fx, double fy, double cx, double cy);

std::vector<cv::Vec3f> convertToVec3fVec(std::vector<cv::Point3f> pVec);

cv::Vec3f convertToVec3f(cv::Point3f &p);

cv::Mat convertToMat(matd_t *m);

cv::Mat convertToMat(std::vector<cv::Point3f> &v);

cv::Mat convertToMat(std::vector<cv::Point2f> &v);

cv::Vec3f convertToVec3f(matd_t *m);

std::vector<cv::Point3f> defineCubeWithPoints(double size = 0.014);

std::vector<cv::Vec3f> defineCubeWithVectors(double side_length);

void drawCube(std::vector<cv::Point3f> &cube, cv::Mat &frame,
              cv::Mat &cameraMatrix, cv::Mat &distortionCoefficients,
              cv::Mat &rotationMatrix, cv::Mat &translationMatrix,
              cv::Scalar &color);

#endif
