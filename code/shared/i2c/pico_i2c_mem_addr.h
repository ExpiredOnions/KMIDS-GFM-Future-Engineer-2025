#pragma once

#include <cstddef>
#include <cstdint>

#include "../types/imu_struct.h"

namespace pico_i2c_mem_addr
{

constexpr size_t MEM_SIZE = 256;

// Command / Status
constexpr size_t COMMAND_SIZE = 1;  // 1 byte
constexpr size_t STATUS_SIZE = 1;   // 1 byte, represented by StatusFlags struct

// IMU data sizes
constexpr size_t ACCEL_DATA_SIZE = sizeof(imu_accel_float_t);
constexpr size_t EULER_ANGLE_SIZE = sizeof(imu_euler_float_t);
constexpr size_t IMU_DATA_SIZE = ACCEL_DATA_SIZE + EULER_ANGLE_SIZE;

// Motor / Steering info
constexpr size_t MOTOR_SPEED_SIZE = sizeof(double);
constexpr size_t STEERING_PERCENT_SIZE = sizeof(float);
constexpr size_t MOVEMENT_INFO_SIZE = MOTOR_SPEED_SIZE + STEERING_PERCENT_SIZE;

// Memory addresses
constexpr size_t COMMAND_ADDR = 0;
constexpr size_t STATUS_ADDR = COMMAND_ADDR + COMMAND_SIZE;
constexpr size_t IMU_DATA_ADDR = STATUS_ADDR + STATUS_SIZE;
constexpr size_t MOVEMENT_INFO_ADDR = IMU_DATA_ADDR + IMU_DATA_SIZE;

static_assert(MOVEMENT_INFO_ADDR + MOVEMENT_INFO_SIZE <= MEM_SIZE, "Memory allocation exceeds buffer size");

// Commands namespace
namespace Command
{

    enum : uint8_t
    {
        NO_COMMAND = 0x00,
        RESTART = 0x01,
        CALIB_NO_OFFSET = 0x02,
        CALIB_WITH_OFFSET = 0x03,
        SKIP_CALIB = 0x04
    };

}

// Status flags as a single byte
struct StatusFlags {
    uint8_t is_running : 1;
    uint8_t imu_ready : 1;
    uint8_t reserved : 6;  // unused bits
};
static_assert(sizeof(StatusFlags) == 1, "StatusFlags must be 1 byte");

}  // namespace pico_i2c_mem_addr
