#include "../include/stream_getter.h"
#include "../include/globals.h"

#include <iostream>

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

// TODO: I don't know why this thread only outputs here and here2 once,
// sometimes not at all. The other thread prints to cout all the time.

int main(int argc, char **argv) {
  StreamGetter stream_getter(0);
  if (!stream_getter.startStream())
    return 1;

  if (!stream_getter.getRetrieved()) {
    return 1;
  }

  // wait until the stream thread starts
  while (!stream_getter.isReady());

  // cv::Mat fr = stream_getter.getFrame();
  // std::cout << "hey\n";
  //
  // cv::imshow("test", fr);
  // std::cout << "bom\n";
  // std::cout << "kok\n";

  while (1) {
    if (!stream_getter.getRetrieved())
      break;

    cv::Mat frame = stream_getter.getFrame();
    if (frame.empty())
      break;
    cv::imshow("frame", frame);
    cv::waitKey(1);
  }

  std::cout << "main quit\n";

  stream_getter.stopStream();

  return 0;
}

