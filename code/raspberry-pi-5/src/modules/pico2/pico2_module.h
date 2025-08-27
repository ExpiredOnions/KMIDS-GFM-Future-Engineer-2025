#pragma once

#include "i2c_master.h"
#include "imu_struct.h"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>

/**
 * @brief High-level interface to the Pico2 module via I2C.
 *
 * Continuously polls IMU and encoder in a background thread.
 * Exposes status flags and thread-safe access to the latest sensor data.
 */
class Pico2Module
{
public:
    explicit Pico2Module(uint8_t i2cAddress = 0x39);
    ~Pico2Module();

    /**
     * @brief Initialize the module and start background polling.
     * @return true if initialization succeeds.
     */
    bool initialize();

    /**
     * @brief Stop background polling and release resources.
     */
    void shutdown();

    /**
     * @brief Check if the Pico2 module is running.
     */
    bool isReady() const;

    /**
     * @brief Check if the IMU has valid data available.
     */
    bool isImuReady() const;

    /**
     * @brief Get latest IMU data (thread-safe).
     * @param[out] outAccel Accelerometer data
     * @param[out] outEuler Euler angles
     * @return true if IMU data is valid
     */
    bool getImuData(ImuAccel &outAccel, ImuEuler &outEuler) const;

    /**
     * @brief Get latest encoder angle (thread-safe).
     * @param[out] outEncoderAngle Encoder angle
     */
    bool getEncoderAngle(double &outEncoderAngle) const;

    /**
     * @brief Set movement info (motor speed and steering)
     * @param motorSpeed [-1.0, 1.0]
     * @param steeringPercent [-100, 100]
     */
    bool setMovementInfo(float motorSpeed, float steeringPercent);

private:
    void pollingLoop();

    I2cMaster master_;
    std::atomic<bool> running_;
    std::thread pollingThread_;

    mutable std::mutex dataMutex_;
    std::condition_variable dataUpdated_;

    pico_i2c_mem_addr::StatusFlags status_;
    ImuAccel latestAccel_;
    ImuEuler latestEuler_;
    double latestEncoderAngle_;
};
