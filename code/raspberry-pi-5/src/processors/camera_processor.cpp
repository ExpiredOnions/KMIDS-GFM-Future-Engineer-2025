#include "camera_processor.h"

namespace camera_processor
{

namespace
{

    // Extract all contours above threshold
    std::vector<ContourInfo> extractContoursInfo(const cv::Mat &mask, double areaThreshold) {
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::vector<ContourInfo> results;
        for (const auto &c : contours) {
            double a = cv::contourArea(c);
            if (a < areaThreshold) continue;

            cv::Moments m = cv::moments(c);
            cv::Point2f centroid(0.0f, 0.0f);
            if (m.m00 != 0.0) {
                centroid = cv::Point2f(static_cast<float>(m.m10 / m.m00), static_cast<float>(m.m01 / m.m00));
            }
            results.push_back({centroid, a});
        }
        return results;
    }

}  // namespace

ColorMasks filterColors(const TimedFrame &timedFrame, double areaThreshold) {
    const cv::Mat &input = timedFrame.frame;

    CV_Assert(!input.empty());
    CV_Assert(input.type() == CV_8UC3);

    cv::Mat hsv;
    cv::cvtColor(input, hsv, cv::COLOR_BGR2HSV);

    // Red
    cv::Mat maskRed1, maskRed2, maskRed;
    cv::inRange(hsv, lowerRed1Light, upperRed1Light, maskRed1);
    cv::inRange(hsv, lowerRed2Light, upperRed2Light, maskRed2);
    cv::bitwise_or(maskRed1, maskRed2, maskRed);

    // Green
    cv::Mat maskGreen1, maskGreen2, maskGreen;
    cv::inRange(hsv, lowerGreen1Light, upperGreen1Light, maskGreen1);
    cv::inRange(hsv, lowerGreen2Light, upperGreen2Light, maskGreen2);
    cv::bitwise_or(maskGreen1, maskGreen2, maskGreen);

    return {{extractContoursInfo(maskRed, areaThreshold), maskRed}, {extractContoursInfo(maskGreen, areaThreshold), maskGreen}};
}

void drawColorMasks(cv::Mat &img, const ColorMasks &colors) {
    CV_Assert(!img.empty());
    CV_Assert(img.type() == CV_8UC3);

    auto drawMaskAndContours = [&](const ColorInfo &colorInfo, const cv::Scalar &maskColor, const cv::Scalar &centroidColor) {
        if (!colorInfo.mask.empty()) {
            // Paint mask pixels
            img.setTo(maskColor, colorInfo.mask);
        }

        // Draw each contour's centroid and text
        for (const auto &c : colorInfo.contours) {
            cv::circle(img, c.centroid, 5, centroidColor, -1);

            std::string text = "A:" + std::to_string(static_cast<int>(c.area)) + " P:(" + std::to_string(static_cast<int>(c.centroid.x)) +
                               "," + std::to_string(static_cast<int>(c.centroid.y)) + ")";
            cv::putText(img, text, c.centroid + cv::Point2f(10, -10), cv::FONT_HERSHEY_SIMPLEX, 0.5, centroidColor, 1);
        }
    };

    // Draw Red and Green
    drawMaskAndContours(colors.red, cv::Scalar(0, 0, 255), cv::Scalar(255, 0, 255));    // Red BGR
    drawMaskAndContours(colors.green, cv::Scalar(0, 255, 0), cv::Scalar(255, 255, 0));  // Green BGR
}

}  // namespace camera_processor
