#ifndef __TAG_POSE_H_
#define __TAG_POSE_H_

#include <iostream>

struct TagPose
{
public:
    int id;
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
    int center_x_px;
    int center_y_px;

    void print()
    {
        std::cout << "tagx: " << x << std::endl;
        std::cout << "tagy: " << y << std::endl;
        std::cout << "tagz: " << z << std::endl;
        std::cout << "tagroll: " << roll << std::endl;
        std::cout << "tagpitch: " << pitch << std::endl;
        std::cout << "tagyaw: " << yaw << std::endl;
        std::cout << "tagcx: " << center_x_px << std::endl;
        std::cout << "tagcy: " << center_y_px << std::endl;
    }
};

#endif