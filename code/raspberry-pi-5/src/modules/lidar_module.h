#pragma once

#include "sl_lidar.h"
#include "sl_lidar_driver.h"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <fstream>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "lidar_struct.h"

/**
 * @brief Lidar module that manages scanning and data acquisition from a SLAMTEC LIDAR device.
 *
 * Runs a background thread that continuously collects LIDAR scan data.
 * Provides thread-safe access to the latest scan points and timestamps.
 * Handles initialization, shutdown, and motor control for the LIDAR.
 */
class LidarModule
{
public:
    /**
     * @brief Construct the Lidar module.
     *
     * Sets up internal state but does not start scanning.
     *
     * @param serialPort Path to the serial port used to communicate with the LIDAR (default "/dev/ttyAMA0").
     * @param baudRate Baud rate for the LIDAR communication (default 460800).
     */
    LidarModule(const char *serialPort = "/dev/ttyAMA0", int baudRate = 460800);

    /**
     * @brief Destroy the Lidar module.
     *
     * Stops scanning, shuts down the device, and releases all resources.
     */
    ~LidarModule();

    /**
     * @brief Initialize the LIDAR driver and connect to the device.
     *
     * @return true if initialization succeeds, false otherwise.
     */
    bool initialize();

    /**
     * @brief Stop the LIDAR and clean up resources.
     */
    void shutdown();

    /**
     * @brief Start LIDAR scanning and begin data acquisition.
     *
     * @return true if scanning starts successfully, false otherwise.
     */
    bool start();

    /**
     * @brief Stop LIDAR scanning and halt the motor.
     */
    void stop();

    /**
     * @brief Get the latest LIDAR scan data.
     *
     * Thread-safe. Copies the latest scan points and timestamp into output parameters.
     *
     * @param[out] outLidarData Vector to receive the latest scan points.
     * @param[out] outTimestamp Timestamp of the latest scan.
     *
     * @return true if data is available, false if no scan has been captured yet.
     */
    bool getData(std::vector<RawLidarNode> &outLidarData, std::chrono::steady_clock::time_point &outTimestamp);

    /**
     * @brief Wait until new LIDAR scan data is available, then return it.
     *
     * This function blocks until new scan points are captured.
     *
     * @param[out] outLidarData Vector to receive the latest scan points.
     * @param[out] outTimestamp Timestamp of the latest scan.
     *
     * @return true if data was successfully retrieved.
     */
    bool waitForData(std::vector<RawLidarNode> &outLidarData, std::chrono::steady_clock::time_point &outTimestamp);

    /**
     * @brief Print information about the connected LIDAR device.
     *
     * @return true if device info was successfully printed.
     */
    bool printDeviceInfo();

    /**
     * @brief Print a vector of LIDAR scan nodes to the console.
     *
     * @param nodeDataVector Vector of RawLidarNode containing the scan data to print.
     */
    static void printScanData(const std::vector<RawLidarNode> &nodeDataVector);

private:
    /**
     * @brief Background thread function for continuous scanning.
     *
     * Captures LIDAR scan frames in a loop and updates the latest data.
     */
    void scanLoop();

    sl::ILidarDriver *lidarDriver_;  ///< Pointer to the LIDAR driver instance.
    sl::IChannel *serialChannel_;    ///< Pointer to the communication channel.
    const char *serialPort_;         ///< Serial port used for LIDAR connection.
    int baudRate_;                   ///< Baud rate for the communication.

    bool initialized_;  ///< True if LIDAR is initialized successfully.

    std::thread lidarThread_;    ///< Thread running the scan loop.
    std::atomic<bool> running_;  ///< Indicates if scanning is active.

    std::mutex lidarDataMutex_;                 ///< Protects access to scan data.
    std::condition_variable lidarDataUpdated_;  ///< Notifies waiting threads of new scan data.

    std::vector<RawLidarNode> latestLidarData_;              ///< Latest scan points.
    std::chrono::steady_clock::time_point latestTimestamp_;  ///< Timestamp of latest scan.
};
