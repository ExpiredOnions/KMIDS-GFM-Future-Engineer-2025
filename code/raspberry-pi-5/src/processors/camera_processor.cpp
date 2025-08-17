#include "camera_processor.h"

namespace camera_processor
{

void filterColors(const TimedFrame &timedFrame, cv::Mat &maskRed, cv::Mat &maskGreen, cv::Mat &maskPink) {
    auto &input = timedFrame.frame;

    CV_Assert(!input.empty());
    CV_Assert(input.type() == CV_8UC3);

    cv::Mat hsv;
    cv::cvtColor(input, hsv, cv::COLOR_BGR2HSV);

    // Red range
    cv::Mat maskRed1, maskRed2;
    cv::inRange(hsv, lowerRed1Light, upperRed1Light, maskRed1);
    cv::inRange(hsv, lowerRed2Light, upperRed2Light, maskRed2);
    cv::bitwise_or(maskRed1, maskRed2, maskRed);

    // Green range
    cv::Mat maskGreen1, maskGreen2;
    cv::inRange(hsv, lowerGreen1Light, upperGreen1Light, maskGreen1);
    cv::inRange(hsv, lowerGreen2Light, upperGreen2Light, maskGreen2);
    cv::bitwise_or(maskGreen1, maskGreen2, maskGreen);

    // Pink range
    cv::Mat maskPink1, maskPink2;
    cv::inRange(hsv, lowerPink1Light, upperPink1Light, maskPink1);
    cv::inRange(hsv, lowerPink2Light, upperPink2Light, maskPink2);
    cv::bitwise_or(maskPink1, maskPink2, maskPink);
}

}  // namespace camera_processor
