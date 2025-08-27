#include "lidar_module.h"
#include "lidar_processor.h"
#include "pico2_module.h"
#include "pico2_processor.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <optional>
#include <thread>

using std::vector;

volatile std::sig_atomic_t stop_flag = 0;

void signalHandler(int signum) {
    std::cout << "\nInterrupt signal (" << signum << ") received.\n";
    stop_flag = 1;
}

std::atomic<bool> paused(false);

void inputThread() {
    std::string command;
    while (true) {
        std::getline(std::cin, command);
        if (command == "p") {
            paused = true;
            std::cout << "Paused\n";
        } else if (command == "r") {
            paused = false;
            std::cout << "Resumed\n";
        } else if (command == "quit") {
            stop_flag = 1;
            break;
        }
    }
}

float perpendicularDistance(float x, float y, float x1, float y1, float x2, float y2) {
    float num = std::fabs((y2 - y1) * x - (x2 - x1) * y + x2 * y1 - y2 * x1);
    float den = std::hypot(x2 - x1, y2 - y1);
    return (den > 1e-6f) ? num / den : 0.0f;
}

int main() {
    std::signal(SIGINT, signalHandler);

    // Initialize LidarModule
    LidarModule lidar;
    if (!lidar.initialize()) return -1;
    lidar.printDeviceInfo();
    if (!lidar.start()) return -1;

    // Initialize Pico2Module
    Pico2Module pico2;
    if (!pico2.initialize()) return -1;

    std::thread(inputThread).detach();

    TimedLidarData timedLidarData;
    TimedPico2Data timedPico2Data;

    const auto loopDuration = std::chrono::milliseconds(16);  // ~60 Hz
    std::optional<float> initialHeading;
    std::optional<RotationDirection> robotTurnDirection;

    while (!stop_flag) {
        auto loopStart = std::chrono::steady_clock::now();

        if (!paused) {
            // --- LIDAR ---
            if (lidar.getData(timedLidarData)) {
                std::cout << "[LIDAR] Nodes: " << timedLidarData.lidarData.size() << std::endl;
            }

            // --- PICO2 ---
            if (pico2.isReady() && pico2.getData(timedPico2Data)) {
                std::cout << "[PICO2] Running: " << pico2.isReady() << ", IMU Ready: " << pico2.isImuReady() << std::endl;

                const auto &accel = timedPico2Data.accel;
                const auto &euler = timedPico2Data.euler;
                const double encoderAngle = timedPico2Data.encoderAngle;

                std::cout << "[IMU] Accel: x=" << accel.x << " y=" << accel.y << " z=" << accel.z << std::endl;
                std::cout << "[IMU] Euler: h=" << euler.h << " r=" << euler.r << " p=" << euler.p << std::endl;
                std::cout << "[ENC] Angle: " << encoderAngle << std::endl;
            }

            // --- Filter LIDAR points ---
            TimedLidarData filteredLidarData;
            filteredLidarData.timestamp = timedLidarData.timestamp;
            for (const auto &node : timedLidarData.lidarData) {
                if (node.distance >= 0.15f) {
                    filteredLidarData.lidarData.push_back(node);
                }
            }

            vector<TimedPico2Data> timedPico2Datas;
            pico2.getAllTimedData(timedPico2Datas);

            // pico2.setMovementInfo(4.5f, 100.0f);

            auto deltaPose = pico2_processor::aproximateRobotPose(filteredLidarData, timedPico2Datas);

            std::cout << "[DeltaPose] ΔX: " << deltaPose.deltaX << " m, ΔY: " << deltaPose.deltaY << " m, ΔH: " << deltaPose.deltaH
                      << " deg" << std::endl;

            // if (!initialHeading) initialHeading = timedPico2Data.euler.h;

            // float heading = timedPico2Data.euler.h - initialHeading.value();
            // heading = std::fmod(heading, 360.0f);
            // if (heading < 0.0f) heading += 360.0f;

            // // --- Wall detection ---
            // auto lineSegments = lidar_processor::getLines(filteredLidarData, 0.05f, 10, 0.10f, 0.10f, 18.0f, 0.20f);

            // // FIXME:
            // auto relativeWalls =
            //     lidar_processor::getRelativeWalls(lineSegments, Direction::fromHeading(heading), heading, 0.30f, 25.0f, 0.22f);

            // // FIXME:
            // if (!robotTurnDirection) {
            //     auto newRobotTurnDirection = lidar_processor::getTurnDirection(relativeWalls);
            //     if (newRobotTurnDirection) robotTurnDirection = newRobotTurnDirection;
            // }

            // auto resolveWalls = lidar_processor::resolveWalls(relativeWalls);

            // if (resolveWalls.frontWall) {
            //     auto frontWall = resolveWalls.frontWall.value();
            //     if (perpendicularDistance(0.0f, 0.0f, frontWall.x1, frontWall.y1, frontWall.x2, frontWall.y2) <= 0.90f) {
            //         pico2.setMovementInfo(1001.0f, 0.0f);
            //     } else {
            //         pico2.setMovementInfo(6.0f, 0.0f);
            //     }
            // } else {
            //     pico2.setMovementInfo(6.0f, 0.0f);
            // }
        }

        // Maintain ~60 Hz loop rate
        auto loopEnd = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(loopEnd - loopStart);
        if (elapsed < loopDuration) {
            std::this_thread::sleep_for(loopDuration - elapsed);
        }
    }
    pico2.setMovementInfo(0.0f, 0.0f);

    // Shutdown
    lidar.stop();
    lidar.shutdown();
    pico2.shutdown();

    std::cout << "[Main] Shutdown complete." << std::endl;
    return 0;
}
