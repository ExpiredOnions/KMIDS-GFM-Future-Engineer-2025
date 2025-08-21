#pragma once

#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

class MotorController
{
public:
    // Constructor: IN1, IN2 for direction, PWM pin for speed
    MotorController(uint pinIn1, uint pinIn2, uint pinPwm);

    void begin();                   // Initialize pins and PWM
    void setSpeed(float speed);     // Speed -100..100 (%)
    void stop(bool brake = false);  // Stop motor (optional brake)

private:
    uint pinIn1_;
    uint pinIn2_;
    uint pinPwm_;
    uint sliceNum_;
    uint channel_;

    void updatePwm(float duty);
};
