#include "i2c_master.h"

#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <wiringPiI2C.h>

I2CMaster::I2CMaster(uint8_t slave_addr)
    : slave_addr_(slave_addr)
    , fd_(-1) {
    fd_ = wiringPiI2CSetup(slave_addr_);
    if (fd_ == -1) {
        printf("Failed to initialize I2C communication with slave 0x%X\n", slave_addr_);
    } else {
        printf("I2C communication initialized with slave 0x%X\n", slave_addr_);
    }
}

I2CMaster::~I2CMaster() {
    // nothing to clean up for wiringPiI2C
}

bool I2CMaster::is_initialized() const {
    return fd_ != -1;
}

bool I2CMaster::write_register(uint8_t reg, const uint8_t *data, size_t len) {
    uint8_t buffer[len + 1];
    buffer[0] = reg;
    memcpy(&buffer[1], data, len);
    return write(fd_, buffer, sizeof(buffer)) != -1;
}

bool I2CMaster::read_register(uint8_t reg, uint8_t *data, size_t len) {
    if (write(fd_, &reg, 1) == -1) return false;
    return read(fd_, data, len) != -1;
}

/** Command operations */
bool I2CMaster::send_command(uint8_t command) {
    uint8_t cmd_buf[2] = {pico_i2c_mem_addr::COMMAND_ADDR, command};
    return write(fd_, cmd_buf, sizeof(cmd_buf)) != -1;
}

bool I2CMaster::read_command(uint8_t &command) {
    uint8_t buf = 0;
    if (!read_register(pico_i2c_mem_addr::COMMAND_ADDR, &buf, 1)) return false;
    command = buf;
    return true;
}

/** Status operations */
bool I2CMaster::read_status(uint8_t &status) {
    return read_register(pico_i2c_mem_addr::STATUS_ADDR, &status, 1);
}

bool I2CMaster::get_is_running(bool &is_running) {
    uint8_t status = 0;
    if (!read_status(status)) return false;
    is_running = (status & 0x01) != 0;
    return true;
}

bool I2CMaster::get_imu_ready(bool &imu_ready) {
    uint8_t status = 0;
    if (!read_status(status)) return false;
    imu_ready = (status & 0x02) != 0;
    return true;
}

/** IMU operations */
bool I2CMaster::read_imu(imu_accel_float_t &accel, imu_euler_float_t &euler) {
    uint8_t buffer[pico_i2c_mem_addr::IMU_DATA_SIZE];
    if (!read_register(pico_i2c_mem_addr::IMU_DATA_ADDR, buffer, sizeof(buffer))) return false;

    memcpy(&accel, buffer, pico_i2c_mem_addr::ACCEL_DATA_SIZE);
    memcpy(&euler, buffer + pico_i2c_mem_addr::ACCEL_DATA_SIZE, pico_i2c_mem_addr::EULER_ANGLE_SIZE);
    return true;
}

/** Movement operations */
bool I2CMaster::read_movement(double &motor_speed, float &steering_percent) {
    uint8_t buffer[pico_i2c_mem_addr::MOVEMENT_INFO_SIZE];
    if (!read_register(pico_i2c_mem_addr::MOVEMENT_INFO_ADDR, buffer, sizeof(buffer))) return false;

    memcpy(&motor_speed, buffer, pico_i2c_mem_addr::MOTOR_SPEED_SIZE);
    memcpy(&steering_percent, buffer + pico_i2c_mem_addr::MOTOR_SPEED_SIZE, pico_i2c_mem_addr::STEERING_PERCENT_SIZE);
    return true;
}
