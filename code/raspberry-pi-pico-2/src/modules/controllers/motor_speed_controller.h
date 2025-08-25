#pragma once

#include <chrono>

#include "encoder_controller.h"
#include "motor_controller.h"
#include "pid_controller.h"

/**
 * @brief Closed-loop speed controller for a DC motor with encoder feedback.
 *
 * Uses a PIDController to adjust motor power and maintain a target speed (RPS).
 */
class MotorSpeedController
{
public:
    /**
     * @brief Construct a new MotorSpeedController object.
     *
     * @param motor Reference to a MotorController instance
     * @param encoder Reference to an EncoderController instance
     * @param Kp Proportional gain for PID
     * @param Ki Integral gain for PID
     * @param Kd Derivative gain for PID
     */
    MotorSpeedController(MotorController &motor, EncoderController &encoder, double Kp = 30.0, double Ki = 30.0, double Kd = 0.04);

    /**
     * @brief Set the desired motor speed in revolutions per second (RPS).
     *
     * @param rps Target speed in RPS
     */
    void setTargetRPS(double rps);

    /**
     * @brief Call this regularly to update motor power based on current speed.
     *
     * Computes the elapsed time automatically.
     */
    void update();

    /**
     * @brief Get the last measured motor speed.
     *
     * @return double Current speed in RPS
     */
    double getCurrentRPS() const;

    /**
     * @brief Get the last applied motor power.
     *
     * @return double Motor power applied (-100 to 100%)
     */
    double getPower() const;

private:
    MotorController &motor_;     /**< Reference to the MotorController used to drive the motor */
    EncoderController &encoder_; /**< Reference to the EncoderController measuring motor rotation */
    PIDController pid_;          /**< PID controller instance for speed regulation */

    double targetRPS_;  /**< Desired motor speed in revolutions per second */
    double currentRPS_; /**< Last measured motor speed in revolutions per second */
    double power_;      /**< Last applied motor power (-100 to 100%) */

    std::chrono::steady_clock::time_point lastUpdateTime_; /**< Timestamp of the last update call */
    double lastAngle_;                                     /**< Last recorded motor angle (degrees) for speed calculation */
};
