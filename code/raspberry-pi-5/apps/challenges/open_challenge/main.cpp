#include "combined_processor.h"
#include "lidar_module.h"
#include "lidar_processor.h"
#include "pico2_module.h"
#include "pid_controller.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <optional>
#include <thread>

volatile std::sig_atomic_t stop_flag = 0;

void signalHandler(int signum) {
    std::cout << "\nInterrupt signal (" << signum << ") received.\n";
    stop_flag = 1;
}

struct State {
    PIDController headingPid{3.0f, 0.0, 0.0f};
    PIDController wallPid{150.0f, 0.0, 0.0f};

    std::optional<float> initialHeading;
    std::optional<RotationDirection> robotTurnDirection;
    Direction headingDirection = Direction::NORTH;
};

void update(float dt, LidarModule &lidar, Pico2Module &pico2, State &state, float &outMotorSpeed, float &outSteeringPercent) {
    std::vector<TimedLidarData> timedLidarDatas;
    lidar.getAllTimedLidarData(timedLidarDatas);
    if (timedLidarDatas.empty()) return;

    std::vector<TimedPico2Data> timedPico2Datas;
    pico2.getAllTimedData(timedPico2Datas);
    if (timedPico2Datas.empty()) return;

    auto &timedLidarData = timedLidarDatas[timedLidarDatas.size() - 1];
    auto &timedPico2Data = timedPico2Datas[timedPico2Datas.size() - 1];

    auto now = std::chrono::steady_clock::now();

    // FIXME:
    // --- Filter LIDAR points ---
    TimedLidarData filteredLidarData;
    filteredLidarData.timestamp = timedLidarData.timestamp;
    for (const auto &node : timedLidarData.lidarData) {
        if (node.distance >= 0.15f) {
            filteredLidarData.lidarData.push_back(node);
        }
    }

    // pico2.setMovementInfo(4.5f, 100.0f);

    auto deltaPose = combined_processor::aproximateRobotPose(filteredLidarData, timedPico2Datas);

    // std::cout << "[DeltaPose] ΔX: " << deltaPose.deltaX << " m, ΔY: " << deltaPose.deltaY << " m, ΔH: " << deltaPose.deltaH << " deg"
    //           << std::endl;

    if (not state.initialHeading) state.initialHeading = timedPico2Data.euler.h;

    float heading = timedPico2Data.euler.h - state.initialHeading.value();
    heading = std::fmod(heading, 360.0f);
    if (heading < 0.0f) heading += 360.0f;

    auto lineSegments = lidar_processor::getLines(filteredLidarData, deltaPose, 0.05f, 10, 0.10f, 0.10f, 18.0f, 0.20f);
    auto relativeWalls = lidar_processor::getRelativeWalls(lineSegments, state.headingDirection, heading, 0.30f, 25.0f, 0.22f);
    auto resolveWalls = lidar_processor::resolveWalls(relativeWalls);

    // Set robotTurnDirection
    if (!state.robotTurnDirection) {
        auto newRobotTurnDirection = lidar_processor::getTurnDirection(relativeWalls);
        if (newRobotTurnDirection) state.robotTurnDirection = newRobotTurnDirection;
    }

    auto frontWall = resolveWalls.frontWall;
    auto backWall = resolveWalls.backWall;

    std::optional<lidar_processor::LineSegment> outerWall;
    std::optional<lidar_processor::LineSegment> innerWall;

    Direction nextHeadingDirection;
    if (state.robotTurnDirection.value_or(RotationDirection::CLOCKWISE) == RotationDirection::CLOCKWISE) {
        outerWall = resolveWalls.leftWall;
        innerWall = resolveWalls.rightWall;

        float nextHeading = state.headingDirection.toHeading() + 90.0f;
        nextHeading = std::fmod(nextHeading + 360.0f, 360.0f);
        nextHeadingDirection = Direction::fromHeading(nextHeading);
    } else {
        outerWall = resolveWalls.rightWall;
        innerWall = resolveWalls.leftWall;

        float nextHeading = state.headingDirection.toHeading() - 90.0f;
        nextHeading = std::fmod(nextHeading + 360.0f, 360.0f);
        nextHeadingDirection = Direction::fromHeading(nextHeading);
    }

    float wallError = 0.0f;
    if (outerWall) {
        wallError = outerWall.value().perpendicularDistance(0.0f, 0.0f) - 0.50f;
    }
    float headingErrorOffset = state.wallPid.update(wallError, dt);

    float headingError = state.headingDirection.toHeading() - heading;
    headingError = std::fmod(headingError + 180.0f, 360.0f);
    if (headingError < 0) headingError += 360.0f;
    headingError -= 180.0f;
    headingError += headingErrorOffset;

    float steeringPercent = state.headingPid.update(headingError, dt);

    std::cout << "Heading: " << heading << "°, Heading Direction: " << state.headingDirection.toHeading()
              << "°, Next Heading Direction: " << nextHeadingDirection.toHeading() << "°, Heading Error: " << headingError << "°"
              << "°, Heading Error Offset: " << headingErrorOffset << "°" << std::endl;

    if (not frontWall) {
        outMotorSpeed = 4.5f;
        outSteeringPercent = steeringPercent;
        return;
    }

    static auto lastTrigger = std::chrono::steady_clock::now() - std::chrono::milliseconds(2000);
    if (frontWall.value().perpendicularDistance(0.0f, 0.0f) <= 0.90f && (now - lastTrigger) >= std::chrono::milliseconds(2000)) {
        outMotorSpeed = 4.5f;
        outSteeringPercent = steeringPercent;

        state.headingDirection = nextHeadingDirection;

        lastTrigger = now;
    } else {
        outMotorSpeed = 4.5f;
        outSteeringPercent = steeringPercent;
    }
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

    State state;

    std::cout << "Waiting 3 seconds before starting control loop..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(3));

    const auto loopDuration = std::chrono::milliseconds(16);  // ~60 Hz
    auto lastTime = std::chrono::steady_clock::now();

    while (!stop_flag) {
        auto loopStart = std::chrono::steady_clock::now();

        std::chrono::duration<float> delta = loopStart - lastTime;
        float dt = delta.count();
        lastTime = loopStart;

        float motorSpeed, steeringPercent;
        update(dt, lidar, pico2, state, motorSpeed, steeringPercent);
        pico2.setMovementInfo(motorSpeed, steeringPercent);

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
