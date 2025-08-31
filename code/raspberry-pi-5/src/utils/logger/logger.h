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

    /**
     * @brief Generate a filename inside a timestamped folder.
     *
     * Creates folder logs/YYYYMMDD_HHMMSS if it doesn't exist.
     * Format: folder/prefix.suffix
     *
     * @param baseFolder Base folder for logs (default: "logs")
     * @param prefix File prefix (default: "log")
     * @param suffix File extension (default: ".bin")
     * @return std::string Full path to the generated file
     */
    static std::string generateFilename(
        const std::string &baseFolder = "logs",
        const std::string &prefix = "log",
        const std::string &suffix = ".bin"
    ) {
        // Generate timestamp for folder
        auto now = std::chrono::system_clock::now();
        auto t_c = std::chrono::system_clock::to_time_t(now);

        std::tm tm_buf;
#if defined(_WIN32) || defined(_WIN64)
        localtime_s(&tm_buf, &t_c);
#else
        localtime_r(&t_c, &tm_buf);
#endif

        std::ostringstream folderName;
        folderName << baseFolder << "/" << std::put_time(&tm_buf, "%Y%m%d_%H%M%S");

        // Create the timestamped folder
        std::filesystem::create_directories(folderName.str());

        // Full file path
        std::ostringstream filePath;
        filePath << folderName.str() << "/" << prefix << suffix;

        return filePath.str();
    }

private:
    std::ofstream file;  ///< Output file stream for the log
    std::mutex mtx;      ///< Mutex for thread-safe writes
};
