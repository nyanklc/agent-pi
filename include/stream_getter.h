#ifndef __STREAM_GETTER_H
#define __STREAM_GETTER_H

#include <chrono>
#include <iostream>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>

#include "globals.h"

class StreamGetter {
   public:
    StreamGetter(const std::string src);

    StreamGetter(const int src);

    bool startStream();

    void stopStream();

    bool isReady();

    bool isUpdated();

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
    bool mUpdated;
};

#endif
