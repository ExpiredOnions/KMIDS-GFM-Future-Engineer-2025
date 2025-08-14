#include <opencv2/opencv.hpp>

#include "lidar_struct.h"

namespace lidar_processor
{

/**
 * @brief Visualize LiDAR scan data as a 2D image.
 *
 * Generates an OpenCV Mat image of the given LiDAR scan, where each point is plotted
 * according to its polar coordinates (angle, distance). The center of the image
 * represents the LiDAR sensor position.
 *
 * @param timedLidarDatas The LiDAR scan data with timestamp.
 * @param size The size of the square output image in pixels (default: 500).
 * @param scale The real-world distance in meters from the center to the edge of the image.
 *              For example, a value of 4.0 means the image covers a radius of 4 meters
 *              from the center to the edge.
 *
 * @return A cv::Mat containing the visualization of the LiDAR scan.
 */
cv::Mat visualizeLidarData(const TimedLidarData &timedLidarDatas, int size = 500, float scale = 4.0f);

}  // namespace lidar_processor
