#include "pico2_module.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

Pico2Module::Pico2Module(uint8_t i2cAddress)
    : master_(i2cAddress)
    , running_(false)
    , latestEncoderAngle_(0.0) {
    status_ = {0};
}

Pico2Module::~Pico2Module() {
    shutdown();
}

bool Pico2Module::initialize() {
    if (!master_.isInitialized()) {
        std::cerr << "[Pico2Module] Failed to initialize I2C master." << std::endl;
        return false;
    }

    running_ = true;
    pollingThread_ = std::thread(&Pico2Module::pollingLoop, this);
    return true;
}

void Pico2Module::shutdown() {
    running_ = false;
    if (pollingThread_.joinable()) {
        pollingThread_.join();
    }
}

bool Pico2Module::isReady() const {
    std::lock_guard<std::mutex> lock(dataMutex_);
    return status_.is_running;
}

bool Pico2Module::isImuReady() const {
    std::lock_guard<std::mutex> lock(dataMutex_);
    return status_.imu_ready;
}

bool Pico2Module::getImuData(ImuAccel &outAccel, ImuEuler &outEuler) const {
    std::lock_guard<std::mutex> lock(dataMutex_);
    if (!status_.imu_ready) return false;

    outAccel = latestAccel_;

    // Convert angles from [-180, 180] to [0, 360]
    auto normalize360 = [](float angle) -> float {
        float a = std::fmod(angle, 360.0f);
        if (a < 0.0f) a += 360.0f;
        return a;
    };

    outEuler.h = normalize360(latestEuler_.h);
    outEuler.r = normalize360(latestEuler_.r);
    outEuler.p = normalize360(latestEuler_.p);

    return true;
}

bool Pico2Module::getEncoderAngle(double &outEncoderAngle) const {
    std::lock_guard<std::mutex> lock(dataMutex_);
    outEncoderAngle = latestEncoderAngle_;
    return true;
}

bool Pico2Module::setMovementInfo(float motorSpeed, float steeringPercent) {
    return master_.writeMovementInfo(motorSpeed, steeringPercent);
}

void Pico2Module::pollingLoop() {
    while (running_) {
        // Read status flags from device
        uint8_t statusByte = 0;
        if (master_.readStatus(statusByte)) {
            std::lock_guard<std::mutex> lock(dataMutex_);
            status_ = *reinterpret_cast<pico_i2c_mem_addr::StatusFlags *>(&statusByte);
        }

        // Read IMU only if imu_ready flag is set
        if (status_.imu_ready) {
            ImuAccel accel;
            ImuEuler euler;
            if (master_.readImu(accel, euler)) {
                std::lock_guard<std::mutex> lock(dataMutex_);
                latestAccel_ = accel;
                latestEuler_ = euler;
            }
        }

        // Always read encoder
        double encoderAngle = 0.0;
        if (master_.readEncoder(encoderAngle)) {
            std::lock_guard<std::mutex> lock(dataMutex_);
            latestEncoderAngle_ = encoderAngle;
        }

        // Notify waiting threads (if any)
        dataUpdated_.notify_all();

        // Poll every 15 ms
        std::this_thread::sleep_for(std::chrono::milliseconds(8));
    }
}
