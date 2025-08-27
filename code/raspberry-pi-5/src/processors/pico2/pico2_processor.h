#pragma once

#include "lidar_struct.h"
#include "pico2_struct.h"
#include "ring_buffer.hpp"

#include <optional>

namespace pico2_processor
{

/**
 * @brief Delta robot pose computed from Pico2 between the latest LIDAR timestamp and previous Pico2 data.
 */
struct RobotDeltaPose {
    float deltaX;  ///< Change in X (meters)
    float deltaY;  ///< Change in Y (meters)
    float deltaH;  ///< Change in heading (degrees, 0-360)
};

/**
 * @brief Approximate the robot's movement since the LIDAR scan using Pico2 data.
 *
 * Loops through the Pico2 samples from the latest sample before the LIDAR timestamp
 * to the most recent, computing the accumulated change in robot pose.
 *
 * @param timedLidarData The LIDAR scan with timestamp.
 * @param timedPico2Datas Time-ordered vector of Pico2 samples.
 * @return RobotDeltaPose containing deltaX, deltaY, and deltaH.
 */
RobotDeltaPose aproximateRobotPose(const TimedLidarData &timedLidarData, const std::vector<TimedPico2Data> &timedPico2Datas);

}  // namespace pico2_processor
