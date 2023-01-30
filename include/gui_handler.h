#ifndef __GUI_HANDLER_H
#define __GUI_HANDLER_H

#include "globals.h"

#include <opencv2/opencv.hpp>

#include <thread>
#include <mutex>
#include <iostream>

class GUIHandler
{
public:
  GUIHandler();

  bool start();

  bool stop();

  bool isReady();

  void setFrame(const cv::Mat frame);

  void show();

private:
  cv::Mat mFrame;
  std::mutex mFrameMutex;
  std::thread mTh;
  bool mReady;
  bool mStopped;
};

#endif
