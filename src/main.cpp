#include "../include/stream_getter.h"
#include "../include/globals.h"
#include "../include/agent.h"
#include "../include/serial_handler.h"
#include "../include/controls.h"

#include <chrono>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <thread>


void calibrate(Agent &agent, StreamGetter &stream_getter) {
  // CALIBRATE
  if (CALIBRATE_MODE) {
    int trials = 0;
    char required;
    std::cout << "start camera calibration? (y/n): ";
    std::cin >> required;
    while (required != 'n') {
      if (required == 'y') {
        cv::Mat fr = stream_getter.getFrameGray();
        if (agent.initFocalLengthCalibration(fr)) {
          if (agent.isCalibrated())
            break;
        } else {
          trials++;
        }
      }
    }
    if (!agent.isCalibrated())
      agent.setFocalLength(FOCAL_LENGTH);
  } else {
    agent.setFocalLength(FOCAL_LENGTH);
  }

  return;
}


int main(int argc, char **argv) {

  std::cout << "initializing\n";
  StreamGetter stream_getter(0);
  Agent agent;
  SerialHandler serial_handler;
  std::cout << "finished initializing\n";

  if (!stream_getter.getRetrieved())
    return 1;

  if (!stream_getter.startStream())
    return 1;

  // wait until the stream thread starts
  while (!stream_getter.isReady());

  calibrate(agent, stream_getter);

  cv::Mat old_frame;

  bool first_run = true;

  while (1) {
    if (!stream_getter.getRetrieved())
      break;

    cv::Mat frame = stream_getter.getFrameGray();
    if (frame.empty())
      break;

    // TODO: do not process the same frame again
    // if (!first_run) {
    //   // skip if frame is a duplicate
    //   cv::Mat comparison;
    //   cv::bitwise_xor(frame, old_frame, comparison);
    //   if (cv::countNonZero(comparison) <= 0) {
    // 	std::cout << "old frame\n";
    // 	old_frame = frame;
    // 	continue;
    //   }
    // }
    // first_run = false;

    // TODO: process
    // std::cout << "processing\n";
    if (agent.process(frame))
      std::cout << "1\n";
    else
     std::cout << "0\n";

    // DEBUG
    // std::cout << "drawing\n";
    agent.drawDetections(frame);

    cv::imshow("frame", frame);
    cv::waitKey(1);
  }

  std::cout << "main quit\n";

  stream_getter.stopStream();

  return 0;
}

