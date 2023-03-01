#include "../include/master_object.h"

void MasterObject::setPosition(std::vector<cv::Point> corners)
{
  cx = 0;
  cy = 0;
  for (auto p : corners)
  {
    cx += p.x;
    cy += p.y;
  }
}
