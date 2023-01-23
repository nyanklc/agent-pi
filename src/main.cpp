#include "../include/stream_getter.h"

#include <iostream>

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

// TODO: I don't know why this thread only outputs here and here2 once, sometimes
// not at all. The other thread prints to cout all the time.

int main(int argc, char **argv) {
  StreamGetter stream_getter(0);
  if (!stream_getter.startStream())
    return 1;

  while (1) {
    if (!stream_getter.getRetrieved())
      break;

    std::cout << std::this_thread::get_id() << " || here\n";
    cv::Mat frame = stream_getter.getFrame();
    if (frame.empty())
      break;
    cv::imshow("frame", frame);
    std::cout << std::this_thread::get_id() << " || here2\n";
  }

  std::cout << "main quit\n";

  stream_getter.stopStream();

  return 0;
}

