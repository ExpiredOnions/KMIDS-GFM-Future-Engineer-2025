#include "lidar_module.h"
#include "lidar_processor.h"

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

    TimedLidarData timedLidarData;
    std::chrono::steady_clock::time_point latestTimestamp;

    std::vector<TimedLidarData> timedLidarDatas;

    if (!lidar.initialize()) return -1;

    lidar.printDeviceInfo();

    if (!lidar.start()) return -1;

    cv::namedWindow("Video", cv::WINDOW_FULLSCREEN);

    std::thread(inputThread).detach();

    while (!stop_flag) {
        if (!paused) {
            if (lidar.getData(timedLidarData)) {
                if (timedLidarData.timestamp != latestTimestamp) {
                    latestTimestamp = timedLidarData.timestamp;

                    // LidarModule::printScanData(timedLidarData.lidarData);
                    cv::Mat lidarMat = lidar_processor::visualizeLidarData(timedLidarData);

                    cv::imshow("Video", lidarMat);
                }
            }

            // if (lidar.getAllTimedLidarData(timedLidarDatas) && lidar.bufferSize() > 9) {
            //     LidarModule::printScanData(timedLidarDatas[0].lidarData);

            //     auto duration = timedLidarDatas[9].timestamp - timedLidarDatas[0].timestamp;
            //     std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() << " ms" << std::endl;
            // }
        }

        // std::this_thread::sleep_for(std::chrono::milliseconds(10));
        cv::waitKey(10);
    }

    lidar.stop();
    lidar.shutdown();

    cv::destroyAllWindows();
}
