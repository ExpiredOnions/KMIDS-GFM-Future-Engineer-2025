#pragma once

#include <pico/types.h>

/**
 * @brief Controller class for a single DC motor using L298N mini with PWM speed control.
 *
 * Allows setting motor speed in both directions and optional braking.
 * Uses hardware PWM on the Raspberry Pi Pico for smooth speed control.
 */
class MotorController
{
public:
    /**
     * @brief Construct a new MotorController object.
     *
     * @param pinIn1 GPIO pin connected to L298N IN1 (PWM & direction)
     * @param pinIn2 GPIO pin connected to L298N IN2 (PWM & direction)
     */
    MotorController(uint pinIn1, uint pinIn2);

    /**
     * @brief Initialize motor control pins and configure PWM.
     *
     * Must be called before using setPower() or stop().
     */
    void begin();

    /**
     * @brief Set motor power.
     *
     * Positive values move the motor forward, negative values reverse it.
     *
     * @param power Motor power percentage (-100.0 to 100.0)
     */
    void setPower(float power);

    /**
     * @brief Stop the motor.
     *
     * Can optionally brake by setting both direction pins high.
     *
     * @param brake If true, apply braking; otherwise, stop freely
     */
    void stop(bool brake = false);

private:
    uint pinIn1_;     /**< GPIO pin for L298N IN1 */
    uint pinIn2_;     /**< GPIO pin for L298N IN2 */
    uint sliceIn1_;   /**< PWM slice number for IN1 */
    uint sliceIn2_;   /**< PWM slice number for IN2 */
    uint channelIn1_; /**< PWM channel number for IN1 */
    uint channelIn2_; /**< PWM channel number for IN2 */

    /**
     * @brief Update the PWM duty cycle on a given pin.
     *
     * @param pin GPIO pin to update
     * @param slice PWM slice number
     * @param channel PWM channel number
     * @param duty Duty cycle as a fraction (0.0 to 1.0)
     */
    void updatePwm(uint pin, uint slice, uint channel, float duty);
};
