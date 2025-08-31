#include "motor_speed_controller.h"

MotorSpeedController::MotorSpeedController(MotorController &motor, EncoderController &encoder, double Kp, double Ki, double Kd)
    : motor_(motor)
    , encoder_(encoder)
    , pid_(Kp, Ki, Kd)
    , targetRPS_(0.0)
    , currentRPS_(0.0)
    , power_(0.0)
    , lastAngle_(0.0) {
    lastUpdateTime_ = std::chrono::steady_clock::now();
}

void MotorSpeedController::setTargetRPS(double rps) {
    targetRPS_ = rps;
}

void MotorSpeedController::update() {
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - lastUpdateTime_).count();
    lastUpdateTime_ = now;

    // Measure speed in RPS
    double angle = encoder_.getAngle();  // degrees
    double deltaAngle = angle - lastAngle_;
    currentRPS_ = (deltaAngle / 360.0) / dt;
    lastAngle_ = angle;

    // Compute PID output
    double error = targetRPS_ - currentRPS_;
    power_ = pid_.update(error, dt);

    if (targetRPS_ > 0.0) {
        // Ensure forward bias
        if (power_ > 0.0 && power_ < 30.0) {
            power_ = 30.0;
        }
    } else if (targetRPS_ < 0.0) {
        // Ensure reverse bias
        if (power_ < 0.0 && power_ > -30.0) {
            power_ = -30.0;
        }
    } else {
        // Target = 0 → full stop
        power_ = 0.0;
    }

    // Apply to motor
    motor_.setPower((float)power_);
}

double MotorSpeedController::getCurrentRPS() const {
    return currentRPS_;
}

double MotorSpeedController::getPower() const {
    return power_;
}
