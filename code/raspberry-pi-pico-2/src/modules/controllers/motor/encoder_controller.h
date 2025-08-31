#pragma once

#include <atomic>
#include <pico/types.h>

/**
 * @brief Controller class for a quadrature rotary encoder on the Raspberry Pi Pico.
 *
 * This class handles a single quadrature encoder using GPIO interrupts.
 * It maintains a thread-safe tick count and provides convenient methods
 * to get the current count or the angle in degrees based on pulses per revolution
 * and optional gear ratio.
 *
 * @note Currently, this class is implemented as a singleton because the
 *       GPIO interrupt service routine requires a static pointer to the instance.
 *       Only one EncoderController instance should be created at a time.
 */
class EncoderController
{
public:
    /**
     * @brief Construct a new EncoderController object.
     *
     * @param pinA GPIO pin connected to encoder channel A
     * @param pinB GPIO pin connected to encoder channel B
     * @param pulsesPerRev Number of encoder pulses per motor revolution
     * @param gearRatio Optional gear ratio (default 1)
     */
    EncoderController(uint pinA, uint pinB, int pulsesPerRev, int gearRatio = 1);

    /**
     * @brief Initialize the encoder GPIO pins and attach interrupts.
     *
     * Must be called before using other methods.
     */
    void begin();

    /**
     * @brief Get the current raw encoder count.
     *
     * @return long The encoder tick count
     */
    long getCount() const;

    /**
     * @brief Get the current angle in degrees.
     *
     * Computes the angle based on pulses per revolution and gear ratio.
     *
     * @return double Current angle in degrees
     */
    double getAngle() const;

    /**
     * @brief Reset the encoder count to zero.
     */
    void reset();

private:
    /**
     * @brief Internal handler for GPIO interrupts.
     *
     * Updates encoder count based on the quadrature signal.
     */
    void handleInterrupt();

    /**
     * @brief Static wrapper for the ISR required by the Pico SDK.
     *
     * Calls the instance's handleInterrupt() method.
     *
     * @param gpio GPIO pin number that triggered the interrupt
     * @param events Event flags (rise/fall)
     */
    static void encoderISR(uint gpio, uint32_t events);

    uint pinA_;        /**< GPIO pin for encoder channel A */
    uint pinB_;        /**< GPIO pin for encoder channel B */
    int pulsesPerRev_; /**< Encoder pulses per revolution */
    int gearRatio_;    /**< Optional gear ratio */

    static EncoderController *instance_; /**< Singleton instance for ISR access */
    volatile int lastEncoded_;           /**< Last encoder state for ISR calculation */
    std::atomic<long> encoderValue_;     /**< Current encoder count (thread-safe) */
};
