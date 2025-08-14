#include "log_reader.h"
#include <iostream>

LogReader::LogReader(const std::string &filename)
    : filePath_(filename) {}

bool LogReader::readAll(std::vector<LogEntry> &entries) {
    std::ifstream file(filePath_, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Failed to open log file: " << filePath_ << std::endl;
        return false;
    }

    entries.clear();

    while (file) {
        uint64_t ts = 0;
        size_t dataSize = 0;

        file.read(reinterpret_cast<char *>(&ts), sizeof(ts));
        if (!file) break;

        file.read(reinterpret_cast<char *>(&dataSize), sizeof(dataSize));
        if (!file) break;

        std::vector<uint8_t> buffer(dataSize);
        file.read(reinterpret_cast<char *>(buffer.data()), dataSize);
        if (!file) break;

        entries.push_back(LogEntry{ts, std::move(buffer)});
    }

    return true;
}
