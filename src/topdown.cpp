#include "../include/topdown.h"

cv::Mat TopDown::prepareView(std::vector<TopDownObject> &objects, cv::Size &window_size) {
    cv::Mat view(window_size, CV_8UC4, cv::Scalar(255, 255, 255, 255));

    for (auto &obj : objects) {
        cv::Scalar obj_color(0, 0, 0);
        if (obj.name == "apriltag")
            obj_color = cv::Scalar(255, 0, 0);

        obj.drawArrow(view, obj_color);
    }

    // draw the camera's pov arrow as well
    cv::Point2f base(view.size().width / 2, 0);
    cv::Point2f head(view.size().width / 2, 60);
    cv::arrowedLine(view, base, head, cv::Scalar(0, 0, 0));
    head.x += 10;
    cv::putText(view, "camera pov", head, cv::HersheyFonts::FONT_ITALIC, 0.4, cv::Scalar(0, 0, 0));
    head.y += -30;
    cv::putText(view, "(width range: 20cm, height range: 2m)", head, cv::HersheyFonts::FONT_ITALIC, 0.4, cv::Scalar(0, 0, 0));

    return view;
}

std::vector<TopDownObject> TopDown::convertToTopDown(std::vector<TagPose> &tag_objects) {
    std::vector<TopDownObject> topdown_objects;
    if (tag_objects.size() == 0) return topdown_objects;  // shouldn't happen
    for (auto &obj : tag_objects) {
        TopDownObject tdo(obj);
        topdown_objects.push_back(tdo);
    }
    return topdown_objects;
}