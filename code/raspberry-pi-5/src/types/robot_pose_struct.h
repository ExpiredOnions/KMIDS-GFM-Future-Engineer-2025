#pragma once

/**
 * @brief Delta robot pose computed from Pico2 between the latest LIDAR timestamp and previous Pico2 data.
 */
struct RobotDeltaPose {
    float deltaX;  ///< Change in X (meters)
    float deltaY;  ///< Change in Y (meters)
    float deltaH;  ///< Change in heading (degrees, 0-360)
};
