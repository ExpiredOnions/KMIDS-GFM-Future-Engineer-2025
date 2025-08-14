#include "lidar_processor.h"

namespace lidar_processor
{

namespace
{

    // --- Helper: Perpendicular distance from point to line ---
    float perpendicularDistance(float x, float y, float x1, float y1, float x2, float y2) {
        float num = std::fabs((y2 - y1) * x - (x2 - x1) * y + x2 * y1 - y2 * x1);
        float den = std::hypot(x2 - x1, y2 - y1);
        return (den > 1e-6f) ? num / den : 0.0f;
    }

    // --- Recursive Split Step ---
    void splitSegment(
        const std::vector<cv::Point2f> &points,
        int start,
        int end,
        float threshold,
        std::vector<LineSegment> &segments,
        int minPointsPerSegment = 10,
        float maxPointGap = 0.20f
    ) {
        if (end <= start + 1) return;

        float x1 = points[start].x;
        float y1 = points[start].y;
        float x2 = points[end].x;
        float y2 = points[end].y;

        // Find farthest point from the segment
        float maxDist = 0.0f;
        int index = -1;
        for (int i = start + 1; i < end; i++) {
            float dist = perpendicularDistance(points[i].x, points[i].y, x1, y1, x2, y2);
            if (dist > maxDist) {
                maxDist = dist;
                index = i;
            }
        }

        if (maxDist > threshold && index != -1) {
            splitSegment(points, start, index, threshold, segments, minPointsPerSegment, maxPointGap);
            splitSegment(points, index, end, threshold, segments, minPointsPerSegment, maxPointGap);
        } else {
            if (end - start >= minPointsPerSegment) {
                // Check that consecutive points are not too far apart
                bool validSegment = true;
                for (int i = start; i < end; i++) {
                    float gap = cv::norm(points[i + 1] - points[i]);
                    if (gap > maxPointGap) {
                        validSegment = false;
                        break;
                    }
                }

                if (validSegment) {
                    segments.push_back({x1, y1, x2, y2});
                }
            }
        }
    }

    // --- Merge Step: Merge collinear & close segments ---
    std::vector<LineSegment> mergeSegments(
        const std::vector<LineSegment> &segments,
        float angleThresholdDeg = 5.0f,
        float gapThreshold = 0.10f
    ) {
        std::vector<LineSegment> merged;
        if (segments.empty()) return merged;

        merged.push_back(segments.front());
        for (size_t i = 1; i < segments.size(); i++) {
            auto &last = merged.back();
            const auto &curr = segments[i];

            // Compute angles
            float angleLast = std::atan2(last.y2 - last.y1, last.x2 - last.x1);
            float angleCurr = std::atan2(curr.y2 - curr.y1, curr.x2 - curr.x1);
            float angleDiff = std::fabs(angleLast - angleCurr) * 180.0f / static_cast<float>(M_PI);

            // If nearly collinear and endpoints are close → merge
            float endGap = std::hypot(curr.x1 - last.x2, curr.y1 - last.y2);
            if (angleDiff < angleThresholdDeg && endGap < gapThreshold) {
                last.x2 = curr.x2;
                last.y2 = curr.y2;
            } else {
                merged.push_back(curr);
            }
        }
        return merged;
    }

}  // namespace

std::vector<LineSegment> splitAndMerge(const TimedLidarData &timedLidarData, float splitThreshold) {
    // Convert polar to Cartesian (in meters)
    std::vector<cv::Point2f> points;
    points.reserve(timedLidarData.lidarData.size());

    for (const auto &node : timedLidarData.lidarData) {
        if (node.distance <= 0) continue;
        float rad = node.angle * static_cast<float>(M_PI) / 180.0f;
        points.emplace_back(node.distance * std::cos(rad), -node.distance * std::sin(rad));
    }

    std::vector<LineSegment> rawSegments;
    if (!points.empty()) {
        splitSegment(points, 0, points.size() - 1, splitThreshold, rawSegments);
    }

    // Merge collinear and close segments
    return mergeSegments(rawSegments);
}

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

void drawLineSegment(cv::Mat &img, const LineSegment &segment, float scale, cv::Scalar color, int thickness) {
    CV_Assert(!img.empty());
    CV_Assert(img.type() == CV_8UC3);  // ensure 3-channel image

    cv::Point center(img.cols / 2, img.rows / 2);

    int x1 = static_cast<int>(center.x + segment.x1 * (img.cols / scale));
    int y1 = static_cast<int>(center.y - segment.y1 * (img.rows / scale));
    int x2 = static_cast<int>(center.x + segment.x2 * (img.cols / scale));
    int y2 = static_cast<int>(center.y - segment.y2 * (img.rows / scale));

    cv::line(img, cv::Point(x1, y1), cv::Point(x2, y2), color, thickness);
}

}  // namespace lidar_processor
