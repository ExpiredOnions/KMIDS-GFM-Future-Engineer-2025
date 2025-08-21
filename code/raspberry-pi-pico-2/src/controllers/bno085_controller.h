#pragma once

#include <cmath>

#include "bno08x.h"
#include "hardware/i2c.h"
#include "imu_struct.h"

/**
 * @brief Wrapper class for the BNO085 IMU sensor.
 *
 * Provides convenient methods for initializing the IMU over I2C,
 * enabling rotation vector and accelerometer data, reading sensor events,
 * and performing tare operations.
 */
class Bno085Controller
{
public:
    /**
     * @brief Construct a new Bno085Controller object.
     *
     * @param i2cPort Pointer to the I2C instance (e.g., i2c0 or i2c1)
     * @param sdaPin SDA GPIO pin number
     * @param sclPin SCL GPIO pin number
     * @param address I2C address of the BNO085 (default 0x4A)
     */
    Bno085Controller(i2c_inst_t *i2cPort, uint sdaPin, uint sclPin, uint8_t address = 0x4a);

    /**
     * @brief Initialize the IMU and I2C communication.
     *
     * Must be called before any other method.
     *
     * @return true if initialization succeeds
     * @return false if IMU is not detected
     */
    bool begin();

    /**
     * @brief Enable the rotation vector sensor.
     *
     * @param interval_ms Data reporting interval in milliseconds (default 10ms)
     * @return true if the sensor was successfully enabled
     * @return false if enabling failed
     */
    bool enableRotation(uint interval_ms = 10);

    /**
     * @brief Enable the accelerometer sensor.
     *
     * @param interval_ms Data reporting interval in milliseconds (default 10ms)
     * @return true if the sensor was successfully enabled
     * @return false if enabling failed
     */
    bool enableAccelerometer(uint interval_ms = 10);

    /**
     * @brief Poll the IMU for new sensor data.
     *
     * Fills the provided TimedImuData struct with the latest accelerometer and/or
     * rotation vector readings, along with a timestamp.
     *
     * @param data Reference to a TimedImuData struct to fill
     * @return true if new data was available
     * @return false if no new data was available
     */
    bool update(TimedImuData &data);

    /**
     * @brief Perform a tare (zeroing) operation on the IMU.
     *
     * @param zAxis If true, tare the Z axis
     * @param basis Tare basis (default: rotation vector)
     * @return true if tare was successful
     * @return false if tare failed
     */
    bool tareNow(bool zAxis = false, sh2_TareBasis_t basis = SH2_TARE_BASIS_ROTATION_VECTOR);

    /**
     * @brief Save the current tare settings to non-volatile memory.
     *
     * @return true if save succeeded
     * @return false if save failed
     */
    bool saveTare();

    /**
     * @brief Clear previously saved tare settings.
     *
     * @return true if clear succeeded
     * @return false if clear failed
     */
    bool clearTare();

private:
    i2c_inst_t *i2c_; /**< Pointer to the I2C instance */
    uint sdaPin_;     /**< SDA GPIO pin */
    uint sclPin_;     /**< SCL GPIO pin */
    uint8_t address_; /**< I2C address of the IMU */
    BNO08x imu_;      /**< Internal BNO08x object */
};
