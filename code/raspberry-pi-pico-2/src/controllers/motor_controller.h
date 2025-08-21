#pragma once

#include <pico/types.h>

/**
 * @brief Controller class for a single DC motor using L298N with PWM speed control.
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
     * @param pinIn1 GPIO pin connected to L298N IN1 (direction)
     * @param pinIn2 GPIO pin connected to L298N IN2 (direction)
     * @param pinPwm GPIO pin connected to L298N EN (PWM speed)
     */
    MotorController(uint pinIn1, uint pinIn2, uint pinPwm);

    /**
     * @brief Initialize motor control pins and configure PWM.
     *
     * Must be called before using setSpeed() or stop().
     */
    void begin();

    /**
     * @brief Set motor speed.
     *
     * Positive values move the motor forward, negative values reverse it.
     *
     * @param speed Motor speed percentage (-100.0 to 100.0)
     */
    void setSpeed(float speed);

    /**
     * @brief Stop the motor.
     *
     * Can optionally brake by setting both direction pins high.
     *
     * @param brake If true, apply braking; otherwise, stop freely
     */
    void stop(bool brake = false);

private:
    uint pinIn1_;   /**< GPIO pin for L298N IN1 */
    uint pinIn2_;   /**< GPIO pin for L298N IN2 */
    uint pinPwm_;   /**< GPIO pin for L298N EN (PWM) */
    uint sliceNum_; /**< PWM slice number */
    uint channel_;  /**< PWM channel number */

    /**
     * @brief Update the PWM duty cycle to control motor speed.
     *
     * @param duty Duty cycle as a fraction (0.0 to 1.0)
     */
    void updatePwm(float duty);
};
