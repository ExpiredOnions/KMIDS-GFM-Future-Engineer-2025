#include <cstring>
#include <iostream>
#include <vector>

#include "camera_processor.h"
#include "camera_struct.h"
#include "imu_struct.h"
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

TimedImuData reconstructTimedImu(const LogEntry &entry) {
    TimedImuData imuData{};

    if (entry.data.size() < sizeof(ImuAccel) + sizeof(ImuEuler)) {
        throw std::runtime_error("Invalid IMU entry data size");
    }

    std::memcpy(&imuData.accel, entry.data.data(), sizeof(ImuAccel));
    std::memcpy(&imuData.euler, entry.data.data() + sizeof(ImuAccel), sizeof(ImuEuler));

    // Convert timestamp in nanoseconds back to steady_clock::time_point
    auto ts = std::chrono::nanoseconds(entry.timestamp);
    imuData.timestamp = std::chrono::steady_clock::time_point(ts);

    return imuData;
}
float wrapHeading(float heading) {
    heading = std::fmod(heading, 360.0f);  // remainder after division
    if (heading < 0) {
        heading += 360.0f;  // keep it positive
    }
    return heading;
}

TimedFrame reconstructTimedFrame(const LogEntry &entry) {
    if (entry.data.empty()) {
        throw std::runtime_error("Empty image entry data");
    }

    // Decode image from memory
    cv::Mat frame = cv::imdecode(entry.data, cv::IMREAD_UNCHANGED);
    if (frame.empty()) {
        throw std::runtime_error("Failed to decode image from entry data");
    }

    // Convert timestamp in nanoseconds back to steady_clock::time_point
    auto ts = std::chrono::nanoseconds(entry.timestamp);
    std::chrono::steady_clock::time_point timestamp(ts);

    return TimedFrame{std::move(frame), timestamp};
}

int main(int argc, char **argv) {
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0] << " <lidar_log_file> <imu_log_file> <camera_file>" << std::endl;
        return 1;
    }

    std::string lidarLogFile = argv[1];
    LogReader lidarReader(lidarLogFile);
    std::vector<LogEntry> lidarEntries;

    if (!lidarReader.readAll(lidarEntries)) {
        std::cerr << "Failed to read log file." << std::endl;
        return 1;
    }

    std::cout << "Loaded " << lidarEntries.size() << " Lidar log entries." << std::endl;

    std::string imuLogFile = argv[2];
    LogReader imuReader(imuLogFile);
    std::vector<LogEntry> imuEntries;

    if (!imuReader.readAll(imuEntries)) {
        std::cerr << "Failed to read IMU log file." << std::endl;
        return 1;
    }

    std::cout << "Loaded " << imuEntries.size() << " IMU log entries." << std::endl;

    const auto &intialImuEntry = imuEntries[0];
    TimedImuData intialTimedImuData = reconstructTimedImu(intialImuEntry);

    std::string cameraLogFile = argv[3];
    LogReader cameraReader(cameraLogFile);
    std::vector<LogEntry> cameraEntries;

    if (!cameraReader.readAll(cameraEntries)) {
        std::cerr << "Failed to read log file." << std::endl;
        return 1;
    }

    std::cout << "Loaded " << cameraEntries.size() << " Camera log entries." << std::endl;

    cv::namedWindow("Lidar View", cv::WINDOW_FULLSCREEN);
    cv::namedWindow("Camera View", cv::WINDOW_FULLSCREEN);

    std::optional<RotationDirection> robotTurnDirection;

    size_t i = 0;
    while (true) {
        int key = cv::waitKey(0);  // waits for key press

        if (key == 27) break;  // ESC to exit

        if (key == 81) {  // left arrow
            if (i > 0) i--;
        } else if (key == 83) {  // right arrow
            if (i < lidarEntries.size() - 1) i++;
        } else {
            continue;
        }

        const auto &lidarEntry = lidarEntries[i];
        TimedLidarData timedLidarData = reconstructTimedLidar(lidarEntry);

        TimedLidarData filteredLidarData;
        filteredLidarData.timestamp = timedLidarData.timestamp;
        for (const auto &node : timedLidarData.lidarData) {
            if (node.distance >= 0.15f) {
                filteredLidarData.lidarData.push_back(node);
            }
        }

        const auto &imuEntry = imuEntries[i];
        TimedImuData timedImuData = reconstructTimedImu(imuEntry);
        timedImuData.euler.h -= intialTimedImuData.euler.h;
        timedImuData.euler.h = wrapHeading(timedImuData.euler.h);
        timedImuData.euler.r -= intialTimedImuData.euler.r;
        timedImuData.euler.r = wrapHeading(timedImuData.euler.h);
        timedImuData.euler.p -= intialTimedImuData.euler.p;
        timedImuData.euler.p = wrapHeading(timedImuData.euler.h);

        float heading = timedImuData.euler.h;

        const auto &cameraEntry = cameraEntries[i];
        TimedFrame timedFrame = reconstructTimedFrame(cameraEntry);

        auto lineSegments = lidar_processor::getLines(filteredLidarData, 0.05f, 10, 0.10f, 0.10f, 18.0f, 0.20f);
        auto relativeWalls = lidar_processor::getRelativeWalls(lineSegments, Direction::fromHeading(heading), heading, 0.30f, 25.0f, 0.22f);

        auto newRobotTurnDirecton = lidar_processor::getTurnDirection(relativeWalls);
        if (newRobotTurnDirecton) robotTurnDirection = newRobotTurnDirecton;

        auto resolveWalls = lidar_processor::resolveWalls(relativeWalls);
        auto parkingWalls = lidar_processor::getParkingWalls(lineSegments, Direction::fromHeading(heading), heading, 0.25f);
        auto trafficLightPoints = lidar_processor::getTrafficLightPoints(filteredLidarData, resolveWalls, robotTurnDirection);

        auto colorMasks = camera_processor::filterColors(timedFrame);

        const float SCALE = 6.0f;

        cv::Mat lidarMat(800, 800, CV_8UC3, cv::Scalar(0, 0, 0));
        lidar_processor::drawLidarData(lidarMat, timedLidarData, SCALE);

        // for (size_t i = 0; i < lineSegments.size(); ++i) {
        //     const auto &lineSegment = lineSegments[i];

        //     std::srand(static_cast<unsigned int>(i));
        //     cv::Scalar color(rand() % 256, rand() % 256, rand() % 256);

        //     // lidar_processor::drawLineSegment(lidarMat, lineSegment, SCALE);
        //     lidar_processor::drawLineSegment(lidarMat, lineSegment, SCALE, color);
        // }

        if (resolveWalls.leftWall) {
            cv::Scalar color(0, 0, 255);
            lidar_processor::drawLineSegment(lidarMat, *resolveWalls.leftWall, SCALE, color);
        }
        if (resolveWalls.rightWall) {
            cv::Scalar color(0, 255, 255);
            lidar_processor::drawLineSegment(lidarMat, *resolveWalls.rightWall, SCALE, color);
        }
        if (resolveWalls.frontWall) {
            cv::Scalar color(0, 255, 0);
            lidar_processor::drawLineSegment(lidarMat, *resolveWalls.frontWall, SCALE, color);
        }
        if (resolveWalls.backWall) {
            cv::Scalar color(255, 255, 0);
            lidar_processor::drawLineSegment(lidarMat, *resolveWalls.backWall, SCALE, color);
        }

        if (resolveWalls.farLeftWall) {
            cv::Scalar color(0, 0, 100);
            lidar_processor::drawLineSegment(lidarMat, *resolveWalls.farLeftWall, SCALE, color);
        }
        if (resolveWalls.farRightWall) {
            cv::Scalar color(0, 100, 100);
            lidar_processor::drawLineSegment(lidarMat, *resolveWalls.farRightWall, SCALE, color);
        }

        for (auto &parkingWall : parkingWalls) {
            cv::Scalar color(146, 22, 199);
            lidar_processor::drawLineSegment(lidarMat, parkingWall, SCALE, color);
        }

        for (auto &trafficLightPoint : trafficLightPoints) {
            lidar_processor::drawTrafficLightPoint(lidarMat, trafficLightPoint, SCALE);
        }

        if (robotTurnDirection) {
            if (*robotTurnDirection == RotationDirection::CLOCKWISE) {
                std::cout << "CLOCKWISE" << std::endl;
            } else if (*robotTurnDirection == RotationDirection::COUNTER_CLOCKWISE) {
                std::cout << "COUNTER_CLOCKWISE" << std::endl;
            }
        } else {
            std::cout << "N/A" << std::endl;
        }

        cv::imshow("Lidar View", lidarMat);

        cv::Mat cameraMat = timedFrame.frame.clone();
        camera_processor::drawColorMasks(cameraMat, colorMasks);

        cv::imshow("Camera View", cameraMat);
    }

    cv::destroyAllWindows();
    return 0;
}
