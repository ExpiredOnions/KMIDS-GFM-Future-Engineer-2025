#include "lidar_processor.h"

namespace lidar_processor
{

void drawLidarData(cv::Mat &img, const TimedLidarData &timedLidarDatas, float scale) {
    CV_Assert(!img.empty());
    CV_Assert(img.type() == CV_8UC3);  // make sure it's a 3-channel color image

    cv::Point center(img.cols / 2, img.rows / 2);

    for (const auto &node : timedLidarDatas.lidarData) {
        if (node.distance <= 0) continue;

        float rad = node.angle * static_cast<float>(CV_PI) / 180.0f;
        int x = static_cast<int>(center.x + node.distance * (img.rows / scale) * std::cos(rad));
        int y = static_cast<int>(center.y + node.distance * (img.rows / scale) * std::sin(rad));

        if (x >= 0 && x < img.cols && y >= 0 && y < img.rows) {
            cv::circle(img, cv::Point(x, y), 1, cv::Scalar(255, 255, 255), -1);
        }
    }

    // Draw LiDAR origin
    cv::circle(img, center, 5, cv::Scalar(168, 12, 173), -1);
}

}  // namespace lidar_processor
