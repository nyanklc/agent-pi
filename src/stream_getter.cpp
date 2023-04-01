#include "../include/stream_getter.h"

#include <chrono>
#include <ctime>
#include <mutex>
#include <opencv2/core/utility.hpp>
#include <thread>

StreamGetter::StreamGetter(const std::string src) {
    mCap = cv::VideoCapture(src);
    // std::cout << "mCap created\n";
    mRetrieved = mCap.read(mFrame);
    // std::cout << "mRetrieved: " << mRetrieved << "\n";;
    // std::cout << "mFrame: " << mFrame << "\n";;
    mStopped = false;
}

StreamGetter::StreamGetter(const int src) {
    mCap = cv::VideoCapture(src);
    // std::cout << "mCap created\n";
    mRetrieved = mCap.read(mFrame);
    // std::cout << "mRetrieved: " << mRetrieved << "\n";;
    // std::cout << "mFrame: " << mFrame << "\n";;
    mStopped = false;
}

bool StreamGetter::startStream() {
    try {
        mTh = std::thread(&StreamGetter::getStream, this);
        return true;
    } catch (...) {
        std::cerr << "Couldn't start a thread for video capture.\n";
        return false;
    }
}

void StreamGetter::getStream() {
    // TODO: add mutex for mStopped
    while (!mStopped) {
        // wait before locking the mutex to not hog the members
        std::this_thread::sleep_for(std::chrono::microseconds(100));

        // abort if previous frame was not retrieved
        if (!mRetrieved) {
            std::cerr << "Couldn't retrieve frame.\n";
            stopStream();
            return;
        }

        // // don't yet read the frame if main did not fetch our latest frame
        // if (mUpdated)
        // {
        //   continue;
        // }

        // read frame from stream
        // auto start = cv::getTickCount();
        std::lock_guard<std::mutex> lock(mStreamMutex);

        mRetrieved = mCap.read(mFrame);
        cv::cvtColor(mFrame, mFrameGray, cv::COLOR_BGRA2GRAY);
        // resize for faster processing
        cv::resize(mFrameGray, mFrameGray, cv::Size(), 0.5, 0.5);
        if (GUI_ON)
            cv::resize(mFrame, mFrame, cv::Size(), 0.5, 0.5);
        // std::cout << std::this_thread::get_id() << " || stream_getter fps: "
        // << cv::getTickFrequency() / (cv::getTickCount() - start) << "\n";
        mReady = true;
        mUpdated = true;
    }
}

void StreamGetter::stopStream() {
    // TODO: add mutex for mStopped
    mStopped = true;
    // std::cout << "before join\n";
    mTh.join();
    // std::cout << "after join\n";
    mCap.release();
}

bool StreamGetter::isReady() { return mReady; }

bool StreamGetter::isUpdated() { return mUpdated; }

cv::Mat StreamGetter::getFrame() {
    // auto start = cv::getTickCount();
    cv::Mat ret;
    std::lock_guard<std::mutex> lock(mStreamMutex);
    ret = mFrame;
    // std::cout << "getting frame fps: " << cv::getTickFrequency() /
    // (cv::getTickCount() - start) << "\n";
    mUpdated = false;
    return ret;
}

cv::Mat StreamGetter::getFrameGray() {
    // auto start = cv::getTickCount();
    cv::Mat ret;
    std::lock_guard<std::mutex> lock(mStreamMutex);
    ret = mFrameGray;
    // std::cout << "getting frame fps: " << cv::getTickFrequency() /
    // (cv::getTickCount() - start) << "\n";
    mUpdated = false;
    return ret;
}

bool StreamGetter::getRetrieved() { return mRetrieved; }
