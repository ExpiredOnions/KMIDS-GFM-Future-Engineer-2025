#include "pico2_module.h"

#include <atomic>
#include <chrono>
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

    while (!stop_flag) {
        if (!paused) {
            bool running = pico2.isReady();
            bool imuReady = pico2.isImuReady();

            std::cout << "Status: running=" << running << ", imu_ready=" << imuReady << std::endl;

            if (imuReady) {
                ImuAccel accel;
                ImuEuler euler;
                if (pico2.getImuData(accel, euler)) {
                    std::cout << "Accel: x=" << accel.x << " y=" << accel.y << " z=" << accel.z << std::endl;
                    std::cout << "Euler: h=" << euler.h << " r=" << euler.r << " p=" << euler.p << std::endl;
                }
            }

            double encoderAngle = 0.0;
            if (pico2.getEncoderAngle(encoderAngle)) {
                std::cout << "Encoder Angle: " << encoderAngle << std::endl;
            }

            // Optional: send zero movement
            pico2.setMovementInfo(0.0f, 0.0f);

            std::cout << "-----------------------------" << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    pico2.shutdown();
    std::cout << "[Main] Pico2 module shutdown.\n";
}
