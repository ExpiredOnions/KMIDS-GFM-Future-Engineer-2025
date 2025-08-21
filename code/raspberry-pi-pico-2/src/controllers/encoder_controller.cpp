#include "encoder_controller.h"

// Static instance pointer
EncoderController *EncoderController::instance_ = nullptr;

// Constructor
EncoderController::EncoderController(uint pinA, uint pinB, int pulsesPerRev, int gearRatio)
    : pinA_(pinA)
    , pinB_(pinB)
    , pulsesPerRev_(pulsesPerRev)
    , gearRatio_(gearRatio)
    , lastEncoded_(0)
    , encoderValue_(0) {}

// Initialize GPIO and interrupts
void EncoderController::begin() {
    instance_ = this;

    // Configure pins
    gpio_init(pinA_);
    gpio_set_dir(pinA_, GPIO_IN);
    gpio_pull_up(pinA_);

    gpio_init(pinB_);
    gpio_set_dir(pinB_, GPIO_IN);
    gpio_pull_up(pinB_);

    // Initialize lastEncoded
    int MSB = gpio_get(pinA_);
    int LSB = gpio_get(pinB_);
    lastEncoded_ = (MSB << 1) | LSB;

    // Setup interrupts
    gpio_set_irq_enabled_with_callback(pinA_, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoderISR);
    gpio_set_irq_enabled(pinB_, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}

// Return raw encoder count
long EncoderController::getCount() const {
    return encoderValue_.load();
}

// Return angle in degrees (float)
double EncoderController::getAngle() const {
    return encoderValue_.load() * (360.0 / (pulsesPerRev_ * gearRatio_));
}

// Reset encoder
void EncoderController::reset() {
    encoderValue_ = 0;
}

// Static ISR wrapper
void EncoderController::encoderISR(uint gpio, uint32_t events) {
    if (instance_) instance_->handleInterrupt();
}

// Actual ISR logic
void EncoderController::handleInterrupt() {
    int MSB = gpio_get(pinA_);
    int LSB = gpio_get(pinB_);

    int encoded = (MSB << 1) | LSB;
    int sum = (lastEncoded_ << 2) | encoded;

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue_--;
    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue_++;

    lastEncoded_ = encoded;
}
