#ifndef __TAG_POSE_H_
#define __TAG_POSE_H_

class TagPose
{
public:
  int id;
  float x;
  float y;
  float z;
  float roll;
  float pitch;
  float yaw;
  bool valid;
};

#endif