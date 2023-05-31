#ifndef __TAG_POSE_H_
#define __TAG_POSE_H_

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
};

#endif