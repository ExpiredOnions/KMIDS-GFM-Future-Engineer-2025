#include "pico2_module.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <thread>

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

int main() {
    std::signal(SIGINT, signalHandler);

    Pico2Module pico2;

    if (!pico2.initialize()) {
        std::cerr << "[Main] Failed to initialize Pico2 module.\n";
        return -1;
    }

    std::cout << "[Main] Pico2 module initialized.\n";
    std::thread(inputThread).detach();

    std::optional<float> initialHeading;

    while (!stop_flag) {
        if (!paused) {
            bool running = pico2.isReady();
            bool imuReady = pico2.isImuReady();

            std::cout << "Status: running=" << running << ", imu_ready=" << imuReady << std::endl;

            TimedPico2Data pico2Data;
            if (pico2.getData(pico2Data)) {
                auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(pico2Data.timestamp.time_since_epoch()).count();

                if (not initialHeading) initialHeading = pico2Data.euler.h;

                float heading = pico2Data.euler.h - initialHeading.value();
                heading = std::fmod(heading + 360.0f, 360.0f);

                std::cout << "Timestamp: " << ms << " ms" << std::endl;
                std::cout << "Accel: x=" << pico2Data.accel.x << " y=" << pico2Data.accel.y << " z=" << pico2Data.accel.z << std::endl;
                std::cout << "Euler: h=" << pico2Data.euler.h << " r=" << pico2Data.euler.r << " p=" << pico2Data.euler.p << std::endl;
                std::cout << "Encoder Angle: " << pico2Data.encoderAngle << std::endl;
                std::cout << "Delta Heading: " << heading << std::endl;
            }

            // Optional: send zero movement
            pico2.setMovementInfo(0.0f, 0.0f);
            // pico2.setMovementInfo(4.5f, 0.0f);

            std::cout << "-----------------------------" << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    pico2.setMovementInfo(0.0f, 0.0f);

    pico2.shutdown();
    std::cout << "[Main] Pico2 module shutdown.\n";
}
