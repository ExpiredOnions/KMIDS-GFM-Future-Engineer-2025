#pragma once

#include <cstdint>
#include <thread>
#include <vector>

/**
 * Structure to hold data for each LIDAR scan node.
 * Contains the angle and distance of the detected point.
 */
struct RawLidarNode {
    float angle;
    float distance;
    uint8_t quality;
};

struct TimedLidarData {
    std::vector<RawLidarNode> lidarData;
    std::chrono::steady_clock::time_point timestamp;
};
