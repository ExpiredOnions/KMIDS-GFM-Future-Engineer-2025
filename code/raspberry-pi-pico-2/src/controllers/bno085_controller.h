#pragma once

#include "bno08x.h"
#include "hardware/i2c.h"
#include "imu_struct.h"  // contains imu_accel_float_t, imu_euler_float_t, TimedImuData
#include "pico/stdlib.h"
#include <chrono>
#include <cmath>
#include <stdio.h>

class Bno085Controller
{
public:
    Bno085Controller(i2c_inst_t *i2cPort, uint sdaPin, uint sclPin, uint8_t address = 0x4a);

    bool begin();                                // Initialize I2C + IMU
    bool enableRotation(uint interval_ms = 10);  // Enable rotation vector
    bool enableAccelerometer(uint interval_ms = 10);

    bool update(TimedImuData &data);  // Poll events, fill struct, return true if new data

    // Tare
    bool tareNow(bool zAxis = false, sh2_TareBasis_t basis = SH2_TARE_BASIS_ROTATION_VECTOR);
    bool saveTare();   // save tare settings
    bool clearTare();  // clear tare settings

private:
    i2c_inst_t *i2c_;
    uint sdaPin_;
    uint sclPin_;
    uint8_t address_;
    BNO08x imu_;
};
