#include "servo_controller.h"

#include <hardware/gpio.h>
#include <hardware/pwm.h>

ServoController::ServoController(uint pin, float maxAngle, uint16_t minPulseUs, uint16_t maxPulseUs, uint16_t freqHz)
    : pin_(pin)
    , minPulseUs_(minPulseUs)
    , maxPulseUs_(maxPulseUs)
    , freqHz_(freqHz)
    , maxAngle_(maxAngle)
    , wrap_(0)
    , slice_(0)
    , channel_(0) {}

void ServoController::begin() {
    gpio_set_function(pin_, GPIO_FUNC_PWM);

    slice_ = pwm_gpio_to_slice_num(pin_);
    channel_ = pwm_gpio_to_channel(pin_);

    // Base clock frequency of Pico
    const float clock_hz = 125000000.0f;

    wrap_ = 65535;
    float divider = clock_hz / (freqHz_ * (wrap_ + 1));

    pwm_set_wrap(slice_, wrap_);
    pwm_set_clkdiv(slice_, divider);

    pwm_set_enabled(slice_, true);
}

void ServoController::setAngle(float angle) {
    if (angle < 0.0f) angle = 0.0f;
    if (angle > maxAngle_) angle = maxAngle_;

    float pulse = minPulseUs_ + (angle / maxAngle_) * (maxPulseUs_ - minPulseUs_);
    updatePwm(static_cast<uint16_t>(pulse));
}

void ServoController::setPulseWidth(uint16_t pulseUs) {
    if (pulseUs < minPulseUs_) pulseUs = minPulseUs_;
    if (pulseUs > maxPulseUs_) pulseUs = maxPulseUs_;
    updatePwm(pulseUs);
}

void ServoController::updatePwm(uint16_t pulseUs) {
    float period_us = 1e6f / freqHz_;

    float duty = static_cast<float>(pulseUs) / period_us;

    uint32_t level = static_cast<uint32_t>(duty * (wrap_ + 1));

    pwm_set_chan_level(slice_, channel_, level);
}
