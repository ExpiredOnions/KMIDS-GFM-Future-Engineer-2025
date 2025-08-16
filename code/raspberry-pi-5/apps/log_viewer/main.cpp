#include <cstring>
#include <iostream>
#include <vector>

#include "lidar_processor.h"
#include "lidar_struct.h"
#include "log_reader.h"

TimedLidarData reconstructTimedLidar(const LogEntry &entry) {
    std::vector<RawLidarNode> nodes(entry.data.size() / sizeof(RawLidarNode));
    if (!entry.data.empty()) {
        std::memcpy(nodes.data(), entry.data.data(), entry.data.size());
    }

    // Convert timestamp in nanoseconds back to steady_clock::time_point
    auto ts = std::chrono::nanoseconds(entry.timestamp);
    std::chrono::steady_clock::time_point timestamp(ts);

    return TimedLidarData{std::move(nodes), timestamp};
}

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <lidar_log_file>" << std::endl;
        return 1;
    }

    std::string logFile = argv[1];
    LogReader reader(logFile);
    std::vector<LogEntry> entries;

    if (!reader.readAll(entries)) {
        std::cerr << "Failed to read log file." << std::endl;
        return 1;
    }

    std::cout << "Loaded " << entries.size() << " log entries." << std::endl;

    cv::namedWindow("Video", cv::WINDOW_FULLSCREEN);

    size_t i = 0;
    while (true) {
        int key = cv::waitKey(0);  // waits for key press

        if (key == 27) break;  // ESC to exit

        if (key == 81) {  // left arrow
            if (i > 0) i--;
        } else if (key == 83) {  // right arrow
            if (i < entries.size() - 1) i++;
        } else {
            continue;
        }

        const auto &entry = entries[i];

        TimedLidarData timedLidarData = reconstructTimedLidar(entry);

        TimedLidarData filteredLidarData;
        filteredLidarData.timestamp = timedLidarData.timestamp;
        for (const auto &node : timedLidarData.lidarData) {
            if (node.distance >= 0.15f) {
                filteredLidarData.lidarData.push_back(node);
            }
        }

        auto lineSegments = lidar_processor::getLines(filteredLidarData, 0.05f, 10, 0.10f, 0.10f, 18.0f, 0.20f);
        lineSegments = lidar_processor::getWalls(lineSegments, 0.30f, 25.0f, 0.22f);

        cv::Mat lidarMat(500, 500, CV_8UC3, cv::Scalar(0, 0, 0));
        lidar_processor::drawLidarData(lidarMat, timedLidarData);

        for (size_t i = 0; i < lineSegments.size(); ++i) {
            const auto &lineSegment = lineSegments[i];

            std::srand(static_cast<unsigned int>(i));
            cv::Scalar color(rand() % 256, rand() % 256, rand() % 256);

            // lidar_processor::drawLineSegment(lidarMat, lineSegment, 4.0f);
            lidar_processor::drawLineSegment(lidarMat, lineSegment, 4.0f, color);
        }

        cv::imshow("Video", lidarMat);
    }

    cv::destroyAllWindows();
    return 0;
}
