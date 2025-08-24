#include <algorithm>
#include <cstring>

#include "i2c_slave.h"

namespace i2c_slave
{

using namespace pico_i2c_mem_addr;

context_t context;

void context_init() {
    std::fill_n(context.mem, MEM_SIZE, 0);
    context.mem_address = 0;
    context.mem_address_written = false;
}

void handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    switch (event) {
    case I2C_SLAVE_RECEIVE:
        if (!context.mem_address_written) {
            // The first byte is always the memory address
            context.mem_address = i2c_read_byte_raw(i2c);
            context.mem_address_written = true;
        } else {
            // Write data to the selected memory address
            context.mem[context.mem_address] = i2c_read_byte_raw(i2c);
            context.mem_address++;
        }
        break;

    case I2C_SLAVE_REQUEST:
        // Send the value at the current memory address
        i2c_write_byte_raw(i2c, context.mem[context.mem_address]);
        context.mem_address++;
        break;

    case I2C_SLAVE_FINISH:
        // Reset for next transaction
        context.mem_address_written = false;
        break;

    default:
        break;
    }
}

/** ---------------- Command Helpers ---------------- */

uint8_t get_command() {
    return context.mem[COMMAND_ADDR];
}

void set_command(uint8_t command) {
    context.mem[COMMAND_ADDR] = command;
}

/** ---------------- Status Helpers ---------------- */

void set_is_running(bool is_running) {
    auto *status = reinterpret_cast<StatusFlags *>(&context.mem[STATUS_ADDR]);
    status->is_running = is_running ? 1 : 0;
}

bool get_is_running() {
    auto *status = reinterpret_cast<StatusFlags *>(&context.mem[STATUS_ADDR]);
    return status->is_running;
}

void set_is_imu_ready(bool ready) {
    auto *status = reinterpret_cast<StatusFlags *>(&context.mem[STATUS_ADDR]);
    status->imu_ready = ready ? 1 : 0;
}

bool get_is_imu_ready() {
    auto *status = reinterpret_cast<StatusFlags *>(&context.mem[STATUS_ADDR]);
    return status->imu_ready;
}

/** ---------------- IMU Helpers ---------------- */

void set_imu_data(const imu_accel_float_t &accel, const imu_euler_float_t &euler) {
    memcpy(&context.mem[IMU_DATA_ADDR], &accel, ACCEL_DATA_SIZE);
    memcpy(&context.mem[IMU_DATA_ADDR + ACCEL_DATA_SIZE], &euler, EULER_ANGLE_SIZE);
}

void get_imu_data(imu_accel_float_t &outAccel, imu_euler_float_t &outEuler) {
    memcpy(&outAccel, &context.mem[IMU_DATA_ADDR], ACCEL_DATA_SIZE);
    memcpy(&outEuler, &context.mem[IMU_DATA_ADDR + ACCEL_DATA_SIZE], EULER_ANGLE_SIZE);
}

/** ---------------- Motor / Steering Helpers ---------------- */

void set_movement_info(double motorSpeed, float steeringPercent) {
    memcpy(&context.mem[MOVEMENT_INFO_ADDR], &motorSpeed, MOTOR_SPEED_SIZE);
    memcpy(&context.mem[MOVEMENT_INFO_ADDR + MOTOR_SPEED_SIZE], &steeringPercent, STEERING_PERCENT_SIZE);
}

void get_movement_info(double &outMotorSpeed, float &outSteeringPercent) {
    memcpy(&outMotorSpeed, &context.mem[MOVEMENT_INFO_ADDR], MOTOR_SPEED_SIZE);
    memcpy(&outSteeringPercent, &context.mem[MOVEMENT_INFO_ADDR + MOTOR_SPEED_SIZE], STEERING_PERCENT_SIZE);
}

}  // namespace i2c_slave
