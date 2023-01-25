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
  while (!mStopped) {
    // wait before locking the mutex to not hog the members
    std::this_thread::sleep_for(std::chrono::microseconds(100));

    // abort if previous frame was not retrieved
    if (!mRetrieved) {
      std::cerr << "Couldn't retrieve frame.\n";
      stopStream();
      return;
    }

    // read frame from stream
    auto start = cv::getTickCount();
    std::lock_guard<std::mutex> lock(mStreamMutex);

    mRetrieved = mCap.read(mFrame);
    cv::cvtColor(mFrame, mFrameGray, cv::COLOR_BGRA2GRAY);
    // std::cout << std::this_thread::get_id() << " || stream_getter fps: " << cv::getTickFrequency() / (cv::getTickCount() - start) << "\n";
    mReady = true;
  }
}

void StreamGetter::stopStream() {
  mStopped = true;
  std::cout << "before join\n";
  mTh.join();
  std::cout << "after join\n";
  mCap.release();
}

bool StreamGetter::isReady() {
  return mReady;
}

cv::Mat StreamGetter::getFrame() {
  cv::Mat ret;
  std::lock_guard<std::mutex> lock(mStreamMutex);
  ret = mFrame;
  return ret;
}

cv::Mat StreamGetter::getFrameGray() {
  cv::Mat ret;
  std::lock_guard<std::mutex> lock(mStreamMutex);
  ret = mFrameGray;
  return ret;
}

bool StreamGetter::getRetrieved() {
  return mRetrieved;
}
