#ifndef __GUI_HANDLER_H
#define __GUI_HANDLER_H

#include <iostream>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <thread>

#include "globals.h"

class GUIHandler {
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
