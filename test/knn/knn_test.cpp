#include "../../include/feature_matcher.h"

#include <string>
#include <iostream>

#include <opencv2/opencv.hpp>

const std::string ref_img_path = 
"/home/nyn/Desktop/means/agent-pi/test/knn/ref_img.jpeg";

const std::string test_img_path = 
"/home/nyn/Desktop/means/agent-pi/test/knn/test9.jpeg";

int main(int argc, char **argv) {
    FeatureMatcher fm;
    fm.init(ref_img_path, 1);

    cv::Mat ref = cv::imread(ref_img_path);
    cv::Mat frame = cv::imread(test_img_path);
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGRA2GRAY);

    
    if (!fm.findObject(frame)) {
        std::cout << "failed to find object\n";
        return 1;
    }

    cv::Mat out = fm.drawMatches(ref, frame);
    cv::resize(out, out, cv::Size(), 0.25, 0.25);
    cv::imshow("matches", out);
    
    out = fm.drawGoodMatches(ref, frame);
    cv::resize(out, out, cv::Size(), 0.25, 0.25);
    cv::imshow("good matches", out);

    std::vector<double> counts = fm.getMatchCounts();
    std::cout << "match count: " << counts[0] << std::endl;
    std::cout << "good match count: " << counts[1] << std::endl;
    cv::waitKey();

    return 0;
}