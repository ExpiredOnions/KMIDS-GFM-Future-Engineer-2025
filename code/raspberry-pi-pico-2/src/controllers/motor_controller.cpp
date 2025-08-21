#include "motor_controller.h"

MotorController::MotorController(uint pinIn1, uint pinIn2, uint pinPwm)
    : pinIn1_(pinIn1)
    , pinIn2_(pinIn2)
    , pinPwm_(pinPwm) {}

void MotorController::begin() {
    // Direction pins
    gpio_init(pinIn1_);
    gpio_set_dir(pinIn1_, GPIO_OUT);
    gpio_put(pinIn1_, 0);

    gpio_init(pinIn2_);
    gpio_set_dir(pinIn2_, GPIO_OUT);
    gpio_put(pinIn2_, 0);

    // PWM pin
    gpio_set_function(pinPwm_, GPIO_FUNC_PWM);
    sliceNum_ = pwm_gpio_to_slice_num(pinPwm_);
    channel_ = pwm_gpio_to_channel(pinPwm_);

    pwm_set_wrap(sliceNum_, 1000);  // 0–1000 duty cycle
    pwm_set_chan_level(sliceNum_, channel_, 0);
    pwm_set_enabled(sliceNum_, true);
}

void MotorController::setSpeed(float speed) {
    if (speed > 100.0f) speed = 100.0f;
    if (speed < -100.0f) speed = -100.0f;

    if (speed >= 0) {
        gpio_put(pinIn1_, 1);
        gpio_put(pinIn2_, 0);
        updatePwm(speed / 100.0f);
    } else {
        gpio_put(pinIn1_, 0);
        gpio_put(pinIn2_, 1);
        updatePwm(-speed / 100.0f);
    }
}

void MotorController::stop(bool brake) {
    if (brake) {
        gpio_put(pinIn1_, 1);
        gpio_put(pinIn2_, 1);
    } else {
        gpio_put(pinIn1_, 0);
        gpio_put(pinIn2_, 0);
    }
    updatePwm(0.0f);
}

void MotorController::updatePwm(float duty) {
    uint level = static_cast<uint>(duty * 1000.0f);
    pwm_set_chan_level(sliceNum_, channel_, level);
}
