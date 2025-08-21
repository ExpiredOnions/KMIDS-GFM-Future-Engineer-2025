#pragma once

#include <cstdint>

#include "imu_struct.h"
#include "pico_i2c_mem_addr.h"

/**
 * @brief Pi-side I2C master to communicate with Pico 2.
 */
class I2CMaster
{
public:
    explicit I2CMaster(uint8_t slave_addr);
    ~I2CMaster();

    bool is_initialized() const;

    /** Command operations */
    bool send_command(uint8_t command);
    bool read_command(uint8_t &command);

    /** Status operations */
    bool read_status(uint8_t &status);
    bool get_is_running(bool &is_running);
    bool get_imu_ready(bool &imu_ready);

    /** IMU operations */
    bool read_imu(imu_accel_float_t &accel, imu_euler_float_t &euler);

    /** Movement operations */
    bool read_movement(double &motor_speed, float &steering_percent);

private:
    int fd_;
    uint8_t slave_addr_;

    bool write_register(uint8_t reg, const uint8_t *data, size_t len);
    bool read_register(uint8_t reg, uint8_t *data, size_t len);
};
