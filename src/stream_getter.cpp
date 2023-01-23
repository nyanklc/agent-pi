#include "../include/stream_getter.h"
#include <ctime>
#include <opencv2/core/utility.hpp>
#include <thread>

std::mutex gStreamMutex;

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
  if (!mStopped) {
    // abort if previous frame was not retrieved
    if (!mRetrieved) {
      std::cerr << "Couldn't retrieve frame.\n";
      stopStream();
      return;
    }
    // read frame from stream
    auto start = cv::getTickCount();
    std::lock_guard<std::mutex> lock(gStreamMutex);
    mRetrieved = mCap.read(mFrame);
    std::cout << std::this_thread::get_id() << " || stream_getter fps: " << cv::getTickFrequency() / (cv::getTickCount() - start) << "\n";
  }
}

void StreamGetter::stopStream() {
  mStopped = true;
  mTh.join();
  mCap.release();
}

cv::Mat StreamGetter::getFrame() {
  std::lock_guard<std::mutex> lock(gStreamMutex);
  return mFrame;
}

bool StreamGetter::getRetrieved() {
  return mRetrieved;
}
