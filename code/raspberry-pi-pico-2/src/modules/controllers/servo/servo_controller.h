#pragma once

#include <pico/types.h>

/**
 * @brief Controller class for a hobby RC servo (or any PWM-controlled actuator).
 *
 * Generates a configurable PWM signal with variable duty cycle corresponding
 * to servo angle or pulse width. By default, it is set up for hobby servos
 * (≈50 Hz, 1–2 ms pulse), but any frequency and pulse range can be specified.
 * Uses hardware PWM on the Raspberry Pi Pico for precise timing.
 */
class ServoController
{
public:
    /**
     * @brief Construct a new ServoController object.
     *
     * @param pin GPIO pin connected to servo signal input
     * @param maxAngle Maximum angle in degrees (default 180°)
     * @param minPulseUs Minimum pulse width in microseconds (default 1000 µs ≈ 0°)
     * @param maxPulseUs Maximum pulse width in microseconds (default 2000 µs ≈ maxAngle)
     * @param freqHz PWM frequency (default 50 Hz for most RC servos, but configurable)
     */
    ServoController(uint pin, float maxAngle = 180.0f, uint16_t minPulseUs = 1000, uint16_t maxPulseUs = 2000, uint16_t freqHz = 50);

    /**
     * @brief Initialize the servo control pin and configure PWM.
     *
     * Must be called before using setAngle() or setPulseWidth().
     */
    void begin();

    /**
     * @brief Set the servo angle.
     *
     * Maps angle (0°–maxAngle) to PWM pulse width (minPulseUs–maxPulseUs).
     * The mapping is linear by default.
     *
     * @param angle Desired servo angle in degrees
     */
    void setAngle(float angle);

    /**
     * @brief Set the PWM pulse width directly (for fine control or non-servo devices).
     *
     * @param pulseUs Pulse width in microseconds
     */
    void setPulseWidth(uint16_t pulseUs);

private:
    uint pin_;            /**< GPIO pin connected to servo signal */
    uint slice_;          /**< PWM slice number */
    uint channel_;        /**< PWM channel number */
    uint16_t minPulseUs_; /**< Minimum pulse width (µs) */
    uint16_t maxPulseUs_; /**< Maximum pulse width (µs) */
    uint16_t freqHz_;     /**< PWM frequency (Hz) */
    float maxAngle_;      /**< Maximum angle in degrees */
    uint32_t wrap_;       /**< PWM counter wrap value */

    /**
     * @brief Update the PWM duty cycle to achieve the desired pulse width.
     *
     * @param pulseUs Desired pulse width in microseconds
     */
    void updatePwm(uint16_t pulseUs);
};
