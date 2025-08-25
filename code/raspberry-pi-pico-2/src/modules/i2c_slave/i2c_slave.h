#pragma once

#include <cstddef>
#include <cstdint>

#include "imu_struct.h"
#include "pico/i2c_slave.h"
#include "pico_i2c_mem_addr.h"

namespace i2c_slave
{

/**
 * @brief Context structure for I2C memory operations.
 *
 * This struct stores the shared I2C memory region and the state of the current
 * memory address transaction. The Raspberry Pi (master) accesses and modifies
 * this memory through I2C transactions handled by the Pico (slave).
 */
struct context_t {
    /** Emulated I2C memory buffer */
    uint8_t mem[pico_i2c_mem_addr::MEM_SIZE];

    /** Current memory address being accessed */
    uint8_t mem_address;

    /** Whether the memory address has been written by the master */
    bool mem_address_written;
};

/**
 * @brief Global context instance used by the I2C slave.
 */
extern context_t context;

/**
 * @brief Initialize context and configure I2C slave handling.
 *
 * This function resets the context state and prepares the I2C slave hardware
 * to receive events from the master device.
 */
void contextInit();

/**
 * @brief Handle I2C slave events.
 *
 * This function should be registered as the I2C event handler. It processes
 * read and write events from the master and updates the context accordingly.
 *
 * @param i2c Pointer to the I2C instance.
 * @param event The I2C slave event type.
 */
void handler(i2c_inst_t *i2c, i2c_slave_event_t event);

/** @name Command Helpers
 *  Functions to set or get the current command.
 */
///@{
/**
 * @brief Retrieve the current command from I2C memory.
 * @return The command byte.
 */
uint8_t getCommand();

/**
 * @brief Set the current command in I2C memory.
 * @param command The command byte to set.
 */
void setCommand(uint8_t command);
///@}

/** @name Status Helpers
 *  Functions to manage device status flags.
 */
///@{
/**
 * @brief Set the "is running" flag.
 * @param isRunning True if the system is running, false otherwise.
 */
void setIsRunning(bool isRunning);

/**
 * @brief Get the "is running" flag.
 * @return True if the system is running, false otherwise.
 */
bool getIsRunning();

/**
 * @brief Set the "IMU ready" flag.
 * @param ready True if IMU is ready, false otherwise.
 */
void setIsImuReady(bool ready);

/**
 * @brief Get the "IMU ready" flag.
 * @return True if IMU is ready, false otherwise.
 */
bool getIsImuReady();
///@}

/** @name IMU Data Helpers
 *  Functions to set and get IMU data.
 */
///@{
/**
 * @brief Store IMU accelerometer and Euler angle data in I2C memory.
 *
 * @param accel Reference to accelerometer data structure
 * @param euler Reference to Euler angle data structure
 */
void setImuData(const ImuAccel &accel, const ImuEuler &euler);

/**
 * @brief Retrieve IMU accelerometer and Euler angle data from I2C memory.
 *
 * @param outAccel Reference to an accelerometer data structure to fill
 * @param outEuler Reference to an Euler angle data structure to fill
 */
void getImuData(ImuAccel &outAccel, ImuEuler &outEuler);
///@}

/** @name Movement Info Helpers
 *  Functions to set and get motor and steering data.
 */
///@{
/**
 * @brief Store motor speed and steering percentage in I2C memory.
 *
 * @param motorSpeed Motor speed as double.
 * @param steeringPercent Steering percentage as float (-100.0 to 100.0).
 */
void setMovementInfo(double motorSpeed, float steeringPercent);

/**
 * @brief Retrieve motor speed and steering percentage from I2C memory.
 *
 * @param outMotorSpeed Reference to a double to store motor speed
 * @param outSteeringPercent Reference to a float to store steering percentage
 */
void getMovementInfo(double &outMotorSpeed, float &outSteeringPercent);
///@}

}  // namespace i2c_slave
