#include "../include/gui_handler.h"

GUIHandler::GUIHandler() { mStopped = false; }

bool GUIHandler::start() {
    if (!GUI_ON) return true;

    try {
        mTh = std::thread(&GUIHandler::show, this);
        return true;
    } catch (...) {
        std::cerr << "Couldn't start a thread for GUI.\n";
        return false;
    }
}

void GUIHandler::show() {
    // TODO: add mutex for mStopped
    while (!mStopped) {
        mReady = true;

        // wait before locking the mutex to not hog the members
        std::this_thread::sleep_for(std::chrono::microseconds(100));

        std::lock_guard<std::mutex> lock(mFrameMutex);

        if (mFrame.empty()) continue;

        cv::imshow("agent-pi", mFrame);
        cv::waitKey(1);
    }
}

bool GUIHandler::isReady() {
    if (!GUI_ON) return true;
    return mReady;
}

bool GUIHandler::stop() {
    if (!GUI_ON) return true;

    // TODO: add mutex for mStopped
    mStopped = true;
    mTh.join();
    return true;
}

void GUIHandler::setFrame(const cv::Mat frame) {
    if (!GUI_ON) return;

    std::lock_guard<std::mutex> lock(mFrameMutex);
    mFrame = frame;
    return;
}