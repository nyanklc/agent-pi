#ifndef __CAMERA_CALIBRATOR_H
#define __CAMERA_CALIBRATOR_H

#include <iostream>

class CameraCalibrator {
  // TODO:
 public:
  double calculateFocalLength(double measured_distance, double real_width,
                              double width_in_rf_image);
};

#endif
