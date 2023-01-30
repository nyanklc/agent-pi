#ifndef __STREAM_GETTER_H
#define __STREAM_GETTER_H

#include "globals.h"

#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <chrono>

#include <opencv2/opencv.hpp>

class StreamGetter
{
public:
  StreamGetter(const std::string src);

  StreamGetter(const int src);

  bool startStream();

  void stopStream();

  bool isReady();

  cv::Mat getFrame();

  cv::Mat getFrameGray();

  bool getRetrieved();

private:
  void getStream();

  cv::VideoCapture mCap;
  bool mRetrieved;
  cv::Mat mFrame;
  cv::Mat mFrameGray;
  bool mStopped;
  std::thread mTh;
  bool mReady;
  std::mutex mStreamMutex;
};

#endif
