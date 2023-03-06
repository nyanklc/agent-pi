#include "../include/topdown.h"

cv::Mat TopDown::test(std::string filepath)
{
  cv::Mat frame = cv::imread(filepath);
  return frame;
}