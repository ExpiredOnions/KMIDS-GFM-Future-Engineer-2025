#pragma once

#include "../shared/types/imu_struct.h"

/**
 * @brief A single Pico2 data sample with timestamp.
 *
 * Each sample contains:
 *  - IMU accelerometer values
 *  - IMU Euler angles (heading normalized to [0,360))
 *  - Encoder angle
 *  - Timestamp (steady_clock, monotonic)
 */
struct TimedPico2Data {
    std::chrono::steady_clock::time_point timestamp;
    ImuAccel accel;
    ImuEuler euler;
    double encoderAngle;
};
