#ifndef __GUI_HANDLER_H
#define __GUI_HANDLER_H

#include <iostream>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>

#include "globals.h"

class GUIHandler {
   public:
    GUIHandler();

    bool start(std::string frame_title);

    bool stop();

    bool isReady();

    void setFrame(const cv::Mat frame);

    void show();

   private:
    cv::Mat mFrame;
    std::string mFrameName;
    std::mutex mFrameMutex;
    std::thread mTh;
    bool mReady;
    bool mStopped;
};

#endif
