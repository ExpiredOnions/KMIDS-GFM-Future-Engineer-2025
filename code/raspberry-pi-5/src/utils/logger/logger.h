#pragma once
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>
/**
 * @class Logger
 * @brief Thread-safe binary logger for various sensor data streams.
 *
 * The Logger class provides a mechanism to serialize and save different types
 * of sensor data into a single binary file. Each log entry contains a type
 * identifier, a timestamp, and the raw payload. Access is synchronized using
 * a mutex to allow safe logging from multiple threads.
 */
class Logger
{
public:
    /**
     * @enum LogType
     * @brief Identifies the type of sensor data being logged.
     */
    enum class LogType : uint8_t
    {
        LIDAR = 1,  ///< LIDAR sensor data
        CAMERA = 2  ///< Camera frame data
    };

    /**
     * @brief Constructs a Logger and opens the output file.
     * @param filename Path to the binary log file.
     * @throws std::runtime_error if the file cannot be opened.
     */
    Logger(const std::string &filename);

    /**
     * @brief Destructor closes the log file.
     */
    ~Logger();

    /**
     * @brief Write a block of raw data with a timestamp.
     *
     * Format:
     * [timestamp: uint64_t][dataSize: size_t][data bytes]
     *
     * @param timestamp_ns Timestamp in nanoseconds
     * @param data Pointer to raw bytes
     * @param dataSize Number of bytes
     */
    void writeData(uint64_t timestamp_ns, const void *data, size_t dataSize);

private:
    std::ofstream file;  ///< Output file stream for the log
    std::mutex mtx;      ///< Mutex for thread-safe writes
};
