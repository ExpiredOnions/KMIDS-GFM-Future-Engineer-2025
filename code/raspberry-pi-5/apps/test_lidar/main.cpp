#include "lidar_module.h"

#include <chrono>
#include <csignal>

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
            stop_flag = true;
            break;
        }
    }
}

int main() {
    std::signal(SIGINT, signalHandler);

    LidarModule lidar;

    std::vector<RawLidarNode> lidarData;
    std::chrono::steady_clock::time_point latestLidarDataTimestamp;

    if (!lidar.initialize()) return -1;

    lidar.printDeviceInfo();

    if (!lidar.start()) return -1;

    std::thread(inputThread).detach();

    while (!stop_flag) {
        if (!paused) {
            if (lidar.getData(lidarData, latestLidarDataTimestamp)) {
                LidarModule::printScanData(lidarData);
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    lidar.stop();
    lidar.shutdown();
}
