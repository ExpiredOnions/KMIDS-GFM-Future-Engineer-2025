#pragma once

#include "camera_struct.h"
#include "lidar_struct.h"
#include "pico2_struct.h"
#include "ring_buffer.hpp"
#include "robot_pose_struct.h"

#include <optional>

namespace combined_processor
{

struct SyncedLidarCamera {
    TimedFrame frame;
    TimedLidarData lidar;
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

/**
 * @brief Synchronize a camera frame with a lidar scan, accounting for delay.
 *
 * @param timedFrames Vector of camera frames (time-sorted).
 * @param timedLidarDatas Vector of lidar scans (time-sorted).
 * @param cameraDelay Camera delay relative to lidar:
 *        - Negative: lidar is slower than camera
 *        - Positive: camera is slower than lidar
 * @return std::optional<SyncedData>
 *         The synchronized result, or nullopt if no match found.
 */
std::optional<SyncedLidarCamera> syncLidarCamera(
    const std::vector<TimedFrame> &timedFrames,
    const std::vector<TimedLidarData> &timedLidarDatas,
    std::chrono::milliseconds cameraDelay
);

}  // namespace combined_processor
