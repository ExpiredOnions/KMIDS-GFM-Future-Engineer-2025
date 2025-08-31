#include <algorithm>
#include <cstring>
#include <hardware/gpio.h>

#include "i2c_slave.h"

namespace i2c_slave
{

using namespace pico_i2c_mem_addr;

context_t context;

void contextInit() {
    std::fill_n(context.mem, MEM_SIZE, 0);
    context.mem_address = 0;
    context.mem_address_written = false;
}

void i2cInit(i2c_inst_t *i2c, uint8_t slaveAddr, uint sdaPin, uint sclPin, uint baudrate) {
    gpio_init(sdaPin);
    gpio_init(sclPin);

    gpio_set_function(sdaPin, GPIO_FUNC_I2C);
    gpio_set_function(sclPin, GPIO_FUNC_I2C);

    gpio_pull_up(sdaPin);
    gpio_pull_up(sclPin);

    i2c_init(i2c, baudrate);

    i2c_slave_init(i2c, slaveAddr, i2c_slave::handler);
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

uint8_t getCommand() {
    return context.mem[COMMAND_ADDR];
}

void setCommand(uint8_t command) {
    context.mem[COMMAND_ADDR] = command;
}

/** ---------------- Status Helpers ---------------- */

void setIsRunning(bool isRunning) {
    auto *status = reinterpret_cast<StatusFlags *>(&context.mem[STATUS_ADDR]);
    status->is_running = isRunning ? 1 : 0;
}

bool getIsRunning() {
    auto *status = reinterpret_cast<StatusFlags *>(&context.mem[STATUS_ADDR]);
    return status->is_running;
}

void setIsImuReady(bool ready) {
    auto *status = reinterpret_cast<StatusFlags *>(&context.mem[STATUS_ADDR]);
    status->imu_ready = ready ? 1 : 0;
}

bool getIsImuReady() {
    auto *status = reinterpret_cast<StatusFlags *>(&context.mem[STATUS_ADDR]);
    return status->imu_ready;
}

/** ---------------- IMU Helpers ---------------- */

void setImuData(const ImuAccel &accel, const ImuEuler &euler) {
    memcpy(&context.mem[IMU_DATA_ADDR], &accel, ACCEL_DATA_SIZE);
    memcpy(&context.mem[IMU_DATA_ADDR + ACCEL_DATA_SIZE], &euler, EULER_ANGLE_SIZE);
}

void getImuData(ImuAccel &outAccel, ImuEuler &outEuler) {
    memcpy(&outAccel, &context.mem[IMU_DATA_ADDR], ACCEL_DATA_SIZE);
    memcpy(&outEuler, &context.mem[IMU_DATA_ADDR + ACCEL_DATA_SIZE], EULER_ANGLE_SIZE);
}

/** ---------------- Encoder Angle Helpers ---------------- */

void setEncoderAngle(double angle) {
    memcpy(&context.mem[ENCODER_ANGLE_ADDR], &angle, ENCODER_ANGLE_SIZE);
}

void getEncoderAngle(double &outAngle) {
    memcpy(&outAngle, &context.mem[ENCODER_ANGLE_ADDR], ENCODER_ANGLE_SIZE);
}

/** ---------------- Motor / Steering Helpers ---------------- */

void setMovementInfo(double motorSpeed, float steeringPercent) {
    memcpy(&context.mem[MOVEMENT_INFO_ADDR], &motorSpeed, MOTOR_SPEED_SIZE);
    memcpy(&context.mem[MOVEMENT_INFO_ADDR + MOTOR_SPEED_SIZE], &steeringPercent, STEERING_PERCENT_SIZE);
}

void getMovementInfo(double &outMotorSpeed, float &outSteeringPercent) {
    memcpy(&outMotorSpeed, &context.mem[MOVEMENT_INFO_ADDR], MOTOR_SPEED_SIZE);
    memcpy(&outSteeringPercent, &context.mem[MOVEMENT_INFO_ADDR + MOTOR_SPEED_SIZE], STEERING_PERCENT_SIZE);
}

}  // namespace i2c_slave
