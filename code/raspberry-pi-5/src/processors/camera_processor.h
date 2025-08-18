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
 * @brief Stores information about a single contour.
 *
 * Contains the geometric centroid and area of a contour.
 * The area is represented as a double to preserve the
 * subpixel precision provided by OpenCV’s cv::contourArea().
 */
struct ContourInfo {
    cv::Point2f centroid;  ///< Centroid of the contour (x, y) in image coordinates.
    double area;           ///< Area of the contour in pixels (double precision).
};

/**
 * @brief Stores information about all valid contours of a single color.
 *
 * Contains:
 * - The binary mask for the color.
 * - A list of contour information (centroid and area) for
 *   contours above the specified area threshold.
 */
struct ColorInfo {
    std::vector<ContourInfo> contours;  ///< All contours that passed the area threshold.
    cv::Mat mask;                       ///< Binary mask representing the color.
};

/**
 * @brief Stores filtered results for multiple colors.
 *
 * Holds the masks and contour information for:
 * - Red
 * - Green
 */
struct ColorMasks {
    ColorInfo red;    ///< Results for red color range.
    ColorInfo green;  ///< Results for green color range.
};

/**
 * @brief Filters an input frame for red, green, and pink colors and extracts contours.
 *
 * The input frame is converted to HSV color space, thresholded into binary masks
 * for each target color (red, green, pink), and processed to extract contour
 * centroids and areas. Contours smaller than the given area threshold are discarded.
 *
 * @param timedFrame Input frame with timestamp and image data.
 * @param areaThreshold Minimum area threshold (in pixels) to filter out small/noisy contours.
 * @return ColorMasks A struct containing masks and contour info for red, green, and pink.
 */
ColorMasks filterColors(const TimedFrame &timedFrame, double areaThreshold = 300.0);

/**
 * @brief Draws ColorMasks on an image.
 *
 * - Overlays the mask for each color with semi-transparent color.
 * - Draws the centroids as circles.
 * - Annotates each centroid with its area and position.
 *
 * @param img The image to draw on (CV_8UC3).
 * @param colors The ColorMasks containing masks and contours.
 */
void drawColorMasks(cv::Mat &img, const ColorMasks &colors);

}  // namespace camera_processor
