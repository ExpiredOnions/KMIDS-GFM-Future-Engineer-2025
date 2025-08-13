#pragma once

/**
 * Structure to hold data for each LIDAR scan node.
 * Contains the angle and distance of the detected point.
 */
#include <cstdint>
struct RawLidarNode {
    float angle;
    float distance;
    uint8_t quality;
};
