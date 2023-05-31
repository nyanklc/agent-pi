#ifndef __TOPDOWN_H_
#define __TOPDOWN_H_

#include <opencv2/opencv.hpp>
#include <string>

#include "tag_pose.h"

class TopDownObject
{
public:
    TopDownObject(TagPose tag_pose)
    {
        name = "apriltag";
        id = tag_pose.id;
        x = -tag_pose.x;
        y = tag_pose.z;
        angle = CV_PI - tag_pose.pitch;
        // std::cout << "converting tagpose, id: " << tag_pose.id << ", x: " << tag_pose.x << ", y: " << tag_pose.y << ", z: " << tag_pose.z << ", roll: " << tag_pose.roll << ", pitch: " << tag_pose.pitch << ", yaw: " << tag_pose.yaw << "\n";
        // std::cout << "created topdownobject, name: " << name << ", id: " << id << ", x: " << x << ", y: " << y << ", angle: " << angle << "\n";
    }

    void drawArrow(cv::Mat &frame, cv::Scalar &color, float length = 0.3, double font_scale = 0.4)
    {
        // center on width
        cv::Point2f base(frame.size().width / 2 + frame.size().width * (x - 0.2) / 2.0, frame.size().height * y / 2.0);
        cv::Point2f head(frame.size().width / 2 + ((x - 0.2) + length * sin(angle)) * frame.size().width / 2.0, (y + length * cos(angle)) * frame.size().height / 2.0);

        // std::cout << "drawing arrow, base: " << base << ", head: " << head << "\n";
        cv::arrowedLine(frame, base, head, color);
        std::string object_text = name + " " + std::to_string(id) + " (" + std::to_string(x) + ", " + std::to_string(y) + ")";
        cv::putText(frame, object_text, base, cv::HersheyFonts::FONT_ITALIC, font_scale, color);
    }

    std::string name;
    int id;
    float x;
    float y;
    float angle;
};

class TopDown
{
public:
    cv::Mat prepareView(std::vector<TopDownObject> &objects, cv::Size &window_size);
    static std::vector<TopDownObject> convertToTopDown(std::vector<TagPose> &tag_objects);
};

#endif