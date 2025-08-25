#include "motor_controller.h"

#include <hardware/gpio.h>
#include <hardware/pwm.h>

MotorController::MotorController(uint pinIn1, uint pinIn2)
    : pinIn1_(pinIn1)
    , pinIn2_(pinIn2) {}

void MotorController::begin() {
    // Set IN1 as PWM
    gpio_set_function(pinIn1_, GPIO_FUNC_PWM);
    sliceIn1_ = pwm_gpio_to_slice_num(pinIn1_);
    channelIn1_ = pwm_gpio_to_channel(pinIn1_);

    pwm_set_wrap(sliceIn1_, 1000);  // 0–1000 duty cycle
    pwm_set_chan_level(sliceIn1_, channelIn1_, 0);
    pwm_set_enabled(sliceIn1_, true);

    // Set IN2 as PWM
    gpio_set_function(pinIn2_, GPIO_FUNC_PWM);
    sliceIn2_ = pwm_gpio_to_slice_num(pinIn2_);
    channelIn2_ = pwm_gpio_to_channel(pinIn2_);

    pwm_set_wrap(sliceIn2_, 1000);
    pwm_set_chan_level(sliceIn2_, channelIn2_, 0);
    pwm_set_enabled(sliceIn2_, true);
}

void MotorController::setPower(float power) {
    if (power > 100.0f) power = 100.0f;
    if (power < -100.0f) power = -100.0f;

    if (power > 0) {
        // Forward: PWM on IN1, IN2 = 0
        updatePwm(pinIn1_, sliceIn1_, channelIn1_, power / 100.0f);
        updatePwm(pinIn2_, sliceIn2_, channelIn2_, 0.0f);
    } else if (power < 0) {
        // Reverse: PWM on IN2, IN1 = 0
        updatePwm(pinIn1_, sliceIn1_, channelIn1_, 0.0f);
        updatePwm(pinIn2_, sliceIn2_, channelIn2_, -power / 100.0f);
    } else {
        // Stop (coast)
        updatePwm(pinIn1_, sliceIn1_, channelIn1_, 0.0f);
        updatePwm(pinIn2_, sliceIn2_, channelIn2_, 0.0f);
    }
}

void MotorController::stop(bool brake) {
    if (brake) {
        // Brake: both high
        updatePwm(pinIn1_, sliceIn1_, channelIn1_, 1.0f);
        updatePwm(pinIn2_, sliceIn2_, channelIn2_, 1.0f);
    } else {
        // Coast: both low
        updatePwm(pinIn1_, sliceIn1_, channelIn1_, 0.0f);
        updatePwm(pinIn2_, sliceIn2_, channelIn2_, 0.0f);
    }
}

void MotorController::updatePwm(uint pin, uint slice, uint channel, float duty) {
    uint level = static_cast<uint>(duty * 1000.0f);
    pwm_set_chan_level(slice, channel, level);
}
