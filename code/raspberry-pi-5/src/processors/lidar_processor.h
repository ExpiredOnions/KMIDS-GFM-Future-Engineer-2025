#include <opencv2/opencv.hpp>
#include <optional>

#include "direction.h"
#include "lidar_struct.h"

namespace lidar_processor
{

struct LineSegment {
    float x1, y1, x2, y2;

    // Compute the Euclidean length of the segment
    float length() const {
        float dx = x2 - x1;
        float dy = y2 - y1;
        return std::hypot(dx, dy);  // sqrt(dx*dx + dy*dy)
    }
};

struct RelativeWalls {
    std::vector<LineSegment> frontWalls;
    std::vector<LineSegment> rightWalls;
    std::vector<LineSegment> backWalls;
    std::vector<LineSegment> leftWalls;
};

std::vector<LineSegment> getLines(
    const TimedLidarData &timedLidarData,
    float splitThreshold = 0.05f,
    int minPoints = 10,
    float maxPointGap = 0.10f,
    float minLength = 0.10f,
    float mergeAngleThreshold = 18.0f,
    float mergeGapThreshold = 0.20f
);

RelativeWalls getRelativeWalls(
    const std::vector<LineSegment> &lineSegments,
    Direction targetDirection,
    float heading,
    float minLength = 0.30f,
    float angleThresholdDeg = 25.0f,
    float collinearThreshold = 0.22f
);

std::optional<RotationDirection> getTurnDirection(const RelativeWalls &walls);

std::vector<LineSegment> getParkingWalls(const std::vector<LineSegment> &lineSegments, float maxLength = 0.25);

/**
 * @brief Draw LiDAR scan points onto an existing image.
 *
 * @param img The cv::Mat to draw on. Must be already allocated with correct size and type.
 * @param timedLidarDatas The LiDAR scan data to visualize.
 * @param scale Meters-to-pixels scaling. For example, scale=4 means 4 meters = img.rows pixels.
 */
void drawLidarData(cv::Mat &img, const TimedLidarData &timedLidarDatas, float scale = 4.0f);

/**
 * @brief Draw a single line segment onto an existing image.
 *
 * @param img The cv::Mat to draw on. Must be already allocated with correct size and type.
 * @param segment The LineSegment to draw.
 * @param scale Meters-to-pixels scaling. For example, scale=4 means 4 meters = img.rows pixels.
 * @param color Line color (default green).
 * @param thickness Line thickness (default 2).
 */
void drawLineSegment(
    cv::Mat &img,
    const LineSegment &segment,
    float scale = 4.0f,
    cv::Scalar color = cv::Scalar(0, 255, 0),
    int thickness = 2
);

}  // namespace lidar_processor
