#include "../include/stream_getter.h"
#include "../include/globals.h"
#include "../include/agent.h"
#include "../include/serial_handler.h"
#include "../include/controls.h"
#include "../include/gui_handler.h"

#include <chrono>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

void calibrate(Agent &agent, StreamGetter &stream_getter)
{
  // CALIBRATE
  if (CALIBRATE_MODE)
  {
    int trials = 0;
    char required;
    std::cout << "start camera calibration? (y/n): ";
    std::cin >> required;
    while (required != 'n')
    {
      if (required == 'y')
      {
        cv::Mat fr = stream_getter.getFrameGray();
        if (agent.initFocalLengthCalibration(fr))
        {
          if (agent.isCalibrated())
            break;
        }
        else
        {
          trials++;
        }
      }
    }
    if (!agent.isCalibrated())
      agent.setFocalLength(FOCAL_LENGTH);
  }
  else
  {
    agent.setFocalLength(FOCAL_LENGTH);
  }

  return;
}

bool initSystem(StreamGetter &stream_getter, Agent &agent, SerialHandler &serial_handler, GUIHandler &gui_handler)
{
  if (!stream_getter.getRetrieved())
    return false;

  if (!stream_getter.startStream())
    return false;

  // wait until the stream thread starts
  while (!stream_getter.isReady());

  if (!gui_handler.start())
    return false;

  // wait until the gui thread starts
  while (!gui_handler.isReady());

  calibrate(agent, stream_getter);

  return true;
}

int main(int argc, char **argv)
{

  StreamGetter stream_getter(0);
  Agent agent;
  SerialHandler serial_handler;
  GUIHandler gui_handler;

  initSystem(stream_getter, agent, serial_handler, gui_handler);

  cv::Mat frame;
  cv::Mat frame_colored;
  bool first_run = true;
  while (1)
  {
    if (!stream_getter.getRetrieved())
      break;

    frame = stream_getter.getFrameGray();
    if (GUI_ON)
      frame_colored = stream_getter.getFrame();
    
    if (frame.empty())
      break;

    // TODO: do not process the same frame again
    
    // process
    if (agent.process(frame) && GUI_ON)
    {
      agent.drawDetections(frame_colored);
      agent.printDetections();
    }    
    if (GUI_ON)
      gui_handler.setFrame(frame_colored);
  }
  
  // yes
  stream_getter.stopStream();
  gui_handler.stop();

  return 0;
}
