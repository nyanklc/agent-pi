#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>

#include "../../include/april_tag_detector.h"

const std::string img_path = "../test/apriltag/test2.png";

int main(int argc, char **argv) {
    std::cout << "before atd\n";
    AprilTagDetector atd;
    std::cout << "after atd\n";

    cv::VideoCapture cap(0);

    while (1) {
        cv::Mat frame;
        cap.read(frame);
        cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
        if (!atd.findObject(frame)) {
            std::cout << "detection failed\n";
        }
        atd.drawDetections(frame);

        for (int i = 0; i < zarray_size(atd.getDetections()); i++) {
            apriltag_detection_t *det;
            zarray_get(atd.getDetections(), i, &det);

            std::cout << "0,0: " << det->p[0][0] << std::endl;
            std::cout << "0,1: " << det->p[0][1] << std::endl;
            std::cout << "1,0: " << det->p[1][0] << std::endl;
            std::cout << "1,1: " << det->p[1][1] << std::endl;
            std::cout << "2,0: " << det->p[2][0] << std::endl;
            std::cout << "2,1: " << det->p[2][1] << std::endl;
            std::cout << "3,0: " << det->p[3][0] << std::endl;
            std::cout << "4,1: " << det->p[3][1] << std::endl;
        }

        cv::imshow("april_tag_test", frame);
        cv::waitKey(1);
    }
}
