#pragma once

#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <atomic>
#include <stdio.h>

class EncoderController
{
public:
    EncoderController(uint pinA, uint pinB, int pulsesPerRev, int gearRatio = 1);

    void begin();             // Initialize GPIO and interrupts
    long getCount() const;    // Current raw encoder value
    double getAngle() const;  // Current angle in degrees
    void reset();             // Reset encoder count

private:
    void handleInterrupt();  // Non-static ISR logic

    static void encoderISR(uint gpio, uint32_t events);  // Static ISR wrapper

    uint pinA_;
    uint pinB_;
    int pulsesPerRev_;
    int gearRatio_;

    static EncoderController *instance_;  // Singleton instance for ISR
    volatile int lastEncoded_;
    std::atomic<long> encoderValue_;
};
