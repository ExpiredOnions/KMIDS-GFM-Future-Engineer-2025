#pragma once
#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

/**
 * @brief Represents a single generic log entry
 */
struct LogEntry {
    uint64_t timestamp;         ///< Timestamp in nanoseconds
    std::vector<uint8_t> data;  ///< Raw data bytes
};

class LogReader
{
public:
    LogReader(const std::string &filename);
    bool readAll(std::vector<LogEntry> &entries);

private:
    std::string filePath_;
};
