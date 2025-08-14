#include "lidar_processor.h"

namespace lidar_processor
{

cv::Mat visualizeLidarData(const TimedLidarData &timedLidarDatas, int size, float scale) {
    cv::Mat img(size, size, CV_8UC3, cv::Scalar(0, 0, 0));

    cv::Point center(size / 2, size / 2);
    for (const auto &node : timedLidarDatas.lidarData) {
        if (node.distance <= 0) continue;

        float rad = node.angle * static_cast<float>(CV_PI) / 180.0f;
        int x = static_cast<int>(center.x + node.distance * (size / scale) * std::cos(rad));
        int y = static_cast<int>(center.y - node.distance * (size / scale) * std::sin(rad));

        cv::circle(img, cv::Point(x, y), 1, cv::Scalar(255, 255, 255), -1);
    }
    return img;
}

}  // namespace lidar_processor
