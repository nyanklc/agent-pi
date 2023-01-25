#include "../include/stream_getter.h"
#include "../include/globals.h"
#include "../include/agent.h"
#include "../include/serial_handler.h"
#include "../include/controls.h"

#include <chrono>
#include <iostream>

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

int main(int argc, char **argv) {

  StreamGetter stream_getter(0);
  Agent agent;
  SerialHandler serial_handler;

  if (!stream_getter.getRetrieved())
    return 1;

  if (!stream_getter.startStream())
    return 1;

  // wait until the stream thread starts
  while (!stream_getter.isReady());

  while (1) {
    if (!stream_getter.getRetrieved())
      break;

    cv::Mat frame = stream_getter.getFrameGray();
    if (frame.empty())
      break;
    cv::imshow("frame", frame);
    cv::waitKey(1);
  }

  std::cout << "main quit\n";

  stream_getter.stopStream();

  return 0;
}

