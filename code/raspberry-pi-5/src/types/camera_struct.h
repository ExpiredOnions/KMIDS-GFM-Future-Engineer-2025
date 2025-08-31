#pragma once

#include <opencv2/opencv.hpp>

struct TimedFrame {
    cv::Mat frame;
    std::chrono::steady_clock::time_point timestamp;
};
