#include "camera_module.h"
#include <iostream>

CameraModule::CameraModule(CameraOptionCallback callback)
    : running_(false) {
    callback(cam_);
}

CameraModule::~CameraModule() {
    if (running_) stop();
}

bool CameraModule::start() {
    if (running_) {
        std::cout << "[CameraModule] Already started." << std::endl;
        return false;
    }

    if (not cam_.startVideo()) {
        return false;
    }

    running_ = true;

    cameraThread_ = std::thread(&CameraModule::captureLoop, this);

    return true;
}

void CameraModule::stop() {
    if (!running_) {
        std::cout << "[CameraModule] Not running." << std::endl;
        return;
    }

    running_ = false;
    if (cameraThread_.joinable()) {
        cameraThread_.join();
    }

    cam_.stopVideo();
}

bool CameraModule::getFrame(cv::Mat &outFrame, std::chrono::steady_clock::time_point &outTimestamp) const {
    std::lock_guard<std::mutex> lock(frameMutex_);
    if (frameBuffer_.empty()) return false;

    TimedFrame timedFrame = frameBuffer_.latest().value();
    outFrame = timedFrame.frame;
    outTimestamp = timedFrame.timestamp;
    return true;
}

size_t CameraModule::bufferSize() const {
    std::lock_guard<std::mutex> lock(frameMutex_);
    return frameBuffer_.size();
}

bool CameraModule::getAllTimedFrame(std::vector<TimedFrame> &outTimedFrames) const {
    std::lock_guard<std::mutex> lock(frameMutex_);

    if (frameBuffer_.empty()) return false;

    outTimedFrames = frameBuffer_.getAll();
    return true;
}

bool CameraModule::waitForFrame(cv::Mat &outFrame, std::chrono::steady_clock::time_point &outTimestamp) {
    std::unique_lock<std::mutex> lock(frameMutex_);
    frameUpdated_.wait(lock, [&] { return !frameBuffer_.empty(); });

    TimedFrame timedFrame = frameBuffer_.latest().value();
    outFrame = timedFrame.frame;
    outTimestamp = timedFrame.timestamp;
    return true;
}

void CameraModule::captureLoop() {
    while (running_) {
        cv::Mat frame;
        if (!cam_.getVideoFrame(frame, 1000)) {
            std::cerr << "[CameraModule] Timeout error" << std::endl;
            continue;
        }

        {
            std::lock_guard<std::mutex> lock(frameMutex_);
            frameBuffer_.push({std::move(frame), std::chrono::steady_clock::now()});
        }

        frameUpdated_.notify_all();
    }
}
