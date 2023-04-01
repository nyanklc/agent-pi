#include "../include/camera_calibrator.h"

double CameraCalibrator::calculateFocalLength(double measured_distance,
                                              double real_width,
                                              double width_in_rf_image) {
    if (width_in_rf_image == -1) {
        std::cout << "unable to calculate focal length, probably no object "
                     "detected\n";
        // hoping actual focal length is not -1 :)
        return -1;
    }

    return width_in_rf_image * measured_distance / real_width;
}
