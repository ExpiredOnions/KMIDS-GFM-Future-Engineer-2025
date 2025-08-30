#include "camera_module.h"

#include <csignal>
#include <iostream>

namespace controls = libcamera::controls;

const uint32_t camWidth = 1296;
const uint32_t camHeight = 972;

volatile std::sig_atomic_t stop_flag = 0;

void signalHandler(int signum) {
    std::cout << "\nInterrupt signal (" << signum << ") received.\n";
    stop_flag = 1;
}

std::atomic<bool> paused(false);

CameraModule *globalCamera = nullptr;

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
        } else {
            // Example: set brightness 0.5
            std::istringstream iss(command);
            std::string cmd, name;
            float value;
            iss >> cmd >> name >> value;
            if (cmd == "set" && !name.empty()) {
                if (globalCamera) {
                    globalCamera->changeSetting([=](lccv::PiCamera &cam) {
                        if (name == "awb_r")
                            cam.options->awb_gain_r = value;
                        else if (name == "awb_b")
                            cam.options->awb_gain_b = value;
                        else if (name == "brightness")
                            cam.options->brightness = value;
                        else if (name == "sharpness")
                            cam.options->sharpness = value;
                        else if (name == "saturation")
                            cam.options->saturation = value;
                        else if (name == "contrast")
                            cam.options->contrast = value;
                        else if (name == "gain")
                            cam.options->gain = value;
                        else
                            std::cout << "Unknown setting: " << name << "\n";
                    });
                    std::cout << "Updated " << name << " to " << value << "\n";
                }
            }
        }
    }
}

int main() {
    std::signal(SIGINT, signalHandler);

    auto cameraOptionCallback = [](lccv::PiCamera &cam) {
        libcamera::ControlList &camControls = cam.getControlList();

        cam.options->video_width = camWidth;
        cam.options->video_height = camHeight;
        cam.options->framerate = 30.0f;

        camControls.set(controls::AnalogueGainMode, controls::AnalogueGainModeEnum::AnalogueGainModeManual);
        camControls.set(controls::ExposureTimeMode, controls::ExposureTimeModeEnum::ExposureTimeModeAuto);
        camControls.set(controls::AwbEnable, false);

        cam.options->awb_gain_r = 0.83;
        cam.options->awb_gain_b = 1.5;

        cam.options->brightness = 0.1;
        cam.options->sharpness = 1;
        cam.options->saturation = 1.5;
        cam.options->contrast = 1;
        cam.options->gain = 5;
    };

    CameraModule camera(cameraOptionCallback);

    TimedFrame timedFrame;
    cv::namedWindow("Video", cv::WINDOW_FULLSCREEN);

    std::vector<TimedFrame> timedFrames;
    cv::namedWindow("Delayed Video", cv::WINDOW_FULLSCREEN);

    camera.start();
    globalCamera = &camera;  // so inputThread can access

    std::thread(inputThread).detach();

    while (!stop_flag) {
        if (!paused) {
            if (camera.getFrame(timedFrame)) {
                cv::imshow("Video", timedFrame.frame);
            }

            if (camera.getAllTimedFrame(timedFrames) && camera.bufferSize() > 29) {
                cv::imshow("Delayed Video", timedFrames[0].frame);

                // auto duration = timedFrames[29].timestamp - timedFrames[0].timestamp;
                // std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() << " ms" << std::endl;
            }
        }

        cv::waitKey(15);
    }

    camera.stop();

    cv::destroyAllWindows();
}
