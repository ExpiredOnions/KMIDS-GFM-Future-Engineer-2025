#include <opencv2/opencv.hpp>

#include "camera_struct.h"

namespace camera_processor
{

// ------------------------ Color Ranges (HSV) ------------------------
const cv::Scalar lowerRed1Light(175, 135, 160);
const cv::Scalar upperRed1Light(180, 205, 255);
// If a second red range is required
const cv::Scalar lowerRed2Light(0, 135, 160);
const cv::Scalar upperRed2Light(2, 205, 255);

const cv::Scalar lowerGreen1Light(55, 70, 120);
const cv::Scalar upperGreen1Light(84, 175, 195);
// If a second green range is required
const cv::Scalar lowerGreen2Light(84, 175, 195);
const cv::Scalar upperGreen2Light(84, 175, 195);

const cv::Scalar lowerPink1Light(165, 244, 200);
const cv::Scalar upperPink1Light(171, 255, 255);
// Optional second pink range
const cv::Scalar lowerPink2Light(171, 255, 255);
const cv::Scalar upperPink2Light(171, 255, 255);

/**
 * @brief Filter an input image into separate masks for red, green, and pink colors.
 *
 * @param input The input BGR image (cv::Mat).
 * @param maskRed Output mask for red pixels (CV_8UC1).
 * @param maskGreen Output mask for green pixels (CV_8UC1).
 * @param maskPink Output mask for pink/magenta pixels (CV_8UC1).
 */
void filterColors(const TimedFrame &timedFrame, cv::Mat &maskRed, cv::Mat &maskGreen, cv::Mat &maskPink);

}  // namespace camera_processor
