#pragma once

#include <thread>

struct ImuAccel {
    float x, y, z;
};

struct ImuEuler {
    float h, r, p;
};

struct TimedImuData {
    ImuAccel accel;
    ImuEuler euler;
    std::chrono::steady_clock::time_point timestamp;
};
