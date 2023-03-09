#include <chrono>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <type_traits>

#include "../include/agent.h"
#include "../include/controls.h"
#include "../include/globals.h"
#include "../include/gui_handler.h"
#include "../include/serial_handler.h"
#include "../include/stream_getter.h"
#include "../include/topdown.h"

void calibrate(Agent &agent, StreamGetter &stream_getter)
{
  // CALIBRATE yeah we removed this
  agent.setFocalLength((CAMERA_FX + CAMERA_FY) / 2);
  return;
}

bool initSystem(StreamGetter &stream_getter, Agent &agent,
                SerialHandler &serial_handler, GUIHandler &gui_handler, GUIHandler &gui_handler2)
{
  if (!stream_getter.getRetrieved())
    return false;

  if (!stream_getter.startStream())
    return false;

  // wait until the stream thread starts
  while (!stream_getter.isReady())
    ;

  if (!gui_handler.start("agent-pi"))
    return false;

  while (!gui_handler.isReady())
    ;

  // topdown
  if (!gui_handler2.start("topdown view"))
    return false;

  while (!gui_handler2.isReady())
    ;

  calibrate(agent, stream_getter);

  return true;
}

int main(int argc, char **argv)
{
  StreamGetter stream_getter(0);
  Agent agent;
  SerialHandler serial_handler;
  GUIHandler gui_handler;
  GUIHandler gui_handler_topdown;
  TopDown topdown;

  initSystem(stream_getter, agent, serial_handler, gui_handler, gui_handler_topdown);

  cv::Mat frame;
  cv::Mat frame_colored;
  bool first_run = true;
  while (1)
  {
    // auto stream_start = cv::getTickCount();
    // std::cout << "start: " << stream_start << "\n";

    // get frame
    // do not process the same frame again
    if (!stream_getter.isUpdated())
      continue;

    // quit if stream wasn't read
    if (!stream_getter.getRetrieved())
      break;

    // auto stream_ready = cv::getTickCount();
    // std::cout << "ready: " << stream_start << "\n";

    frame = stream_getter.getFrameGray();
    if (GUI_ON)
      frame_colored = stream_getter.getFrame();

    if (frame.empty())
      break;
    // std::cout << "start @ " << (cv::getTickCount() - stream_start)/ cv::getTickFrequency() << " fps\n";
    // std::cout << "ready @ " << (cv::getTickCount() - stream_ready)/ cv::getTickFrequency() << "fps\n";

    // process
    if (agent.process(frame) && GUI_ON)
    {
      agent.drawDetections(frame_colored, DRAW_CUBES, DRAW_AXES);
      // agent.printDetections();
    }

    if (GUI_ON)
    {
      cv::resize(frame_colored, frame_colored, cv::Size(), 2, 2);
      gui_handler.setFrame(frame_colored);
      cv::Mat f = topdown.test("../img/ref.jpeg");
      gui_handler_topdown.setFrame(f);
    }
  }

  // yes
  stream_getter.stopStream();
  gui_handler.stop();
  gui_handler_topdown.stop();

  return 0;
}
