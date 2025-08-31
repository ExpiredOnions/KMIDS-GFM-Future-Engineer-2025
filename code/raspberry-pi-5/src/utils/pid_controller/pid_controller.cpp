#include "pid_controller.h"

#include <algorithm>

PIDController::PIDController(double Kp, double Ki, double Kd, double outputMin, double outputMax)
    : Kp_(Kp)
    , Ki_(Ki)
    , Kd_(Kd)
    , integral_(0.0)
    , lastError_(0.0)
    , outputMin_(outputMin)
    , outputMax_(outputMax) {}

double PIDController::update(double error, double dt) {
    integral_ += error * dt;
    double derivative = (error - lastError_) / dt;
    lastError_ = error;

    double output = Kp_ * error + Ki_ * integral_ + Kd_ * derivative;
    return std::clamp(output, outputMin_, outputMax_);
}

void PIDController::reset() {
    integral_ = 0.0;
    lastError_ = 0.0;
}
