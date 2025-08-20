#pragma once

#include <thread>

struct imu_accel_float_t {
    float x, y, z;
};

struct imu_euler_float_t {
    float h, r, p;
};

struct TimedImuData {
    imu_accel_float_t accel;
    imu_euler_float_t euler;
    std::chrono::steady_clock::time_point timestamp;
};
