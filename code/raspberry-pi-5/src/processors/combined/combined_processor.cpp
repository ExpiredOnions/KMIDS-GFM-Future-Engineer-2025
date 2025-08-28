#include "combined_processor.h"

#include <cmath>
#include <iostream>

namespace combined_processor
{

constexpr float WHEEL_DIAMETER = 0.055f;

RobotDeltaPose aproximateRobotPose(const TimedLidarData &timedLidarData, const std::vector<TimedPico2Data> &timedPico2Datas) {
    RobotDeltaPose deltaPose{0.0f, 0.0f, 0.0f};

    if (timedPico2Datas.empty()) return deltaPose;

    // Find index of latest Pico2 sample before LIDAR timestamp
    int index = -1;
    for (int i = static_cast<int>(timedPico2Datas.size()) - 1; i >= 0; --i) {
        if (timedPico2Datas[i].timestamp <= timedLidarData.timestamp) {
            index = i;
            break;  // stop at first valid sample
        }
    }
    if (index < 0) return deltaPose;

    float lastHeading = timedPico2Datas[index].euler.h;

    // Initialize robot pose relative to LIDAR timestamp
    float totalDeltaX = 0.0f;
    float totalDeltaY = 0.0f;
    float totalDeltaH = 0.0f;

    // Loop from index+1 to latest Pico2 data
    for (size_t i = index + 1; i < timedPico2Datas.size(); ++i) {
        const auto &prev = timedPico2Datas[i - 1];
        const auto &curr = timedPico2Datas[i];

        // Calculate heading difference
        float dHeading = curr.euler.h - prev.euler.h;
        dHeading = std::fmod(dHeading, 360.0f);
        if (dHeading < 0.0f) dHeading += 360.0f;

        totalDeltaH += dHeading;
        totalDeltaH = std::fmod(totalDeltaH, 360.0f);
        if (totalDeltaH < 0.0f) totalDeltaH += 360.0f;

        // Calculate encoder delta distance
        double dDistance = (curr.encoderAngle - prev.encoderAngle) * (M_PI * WHEEL_DIAMETER) / 360.0;

        // Convert relative movement to x/y
        float headingRad = dHeading * M_PI / 180.0f;
        totalDeltaX += dDistance * std::sin(headingRad);
        totalDeltaY += dDistance * std::cos(headingRad);
    }

    deltaPose.deltaX = totalDeltaX;
    deltaPose.deltaY = totalDeltaY;
    deltaPose.deltaH = totalDeltaH;

    return deltaPose;
}

}  // namespace combined_processor
