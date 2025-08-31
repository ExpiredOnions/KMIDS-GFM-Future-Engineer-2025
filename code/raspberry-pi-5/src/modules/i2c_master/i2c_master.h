#pragma once

#include <cstdint>

#include "imu_struct.h"
#include "pico_i2c_mem_addr.h"

/**
 * @brief Pi-side I2C master to communicate with Pico 2.
 *
 * Provides functions to read/write IMU data, encoder angle, motor speed,
 * steering, and commands/status via I2C.
 */
class I2cMaster
{
public:
    explicit I2cMaster(uint8_t slave_addr);
    ~I2cMaster();

    /**
     * @brief Check if the I2C device was successfully initialized.
     * @return True if initialized, false otherwise.
     */
    bool isInitialized() const;

    /** @name Command operations */
    ///@{
    /**
     * @brief Send a command byte to the slave.
     * @param command Command to send.
     * @return True if successful, false otherwise.
     */
    bool sendCommand(uint8_t command);

    /**
     * @brief Read the current command byte from the slave.
     * @param outCommand Reference to store the read command.
     * @return True if successful, false otherwise.
     */
    bool readCommand(uint8_t &outCommand);
    ///@}

    /** @name Status operations */
    ///@{
    /**
     * @brief Read a status byte from the slave.
     * @param outStatus Reference to store the status.
     * @return True if successful, false otherwise.
     */
    bool readStatus(uint8_t &outStatus);

    /**
     * @brief Check whether the slave is running.
     * @param outIsRunning Reference to store the running state.
     * @return True if successful, false otherwise.
     */
    bool getIsRunning(bool &outIsRunning);

    /**
     * @brief Check whether the IMU is ready.
     * @param outImuReady Reference to store IMU ready state.
     * @return True if successful, false otherwise.
     */
    bool getImuReady(bool &outImuReady);
    ///@}

    /** @name IMU operations */
    ///@{
    /**
     * @brief Read IMU data from the slave.
     * @param outAccel Reference to store accelerometer data.
     * @param outEuler Reference to store Euler angles.
     * @return True if successful, false otherwise.
     */
    bool readImu(ImuAccel &outAccel, ImuEuler &outEuler);

    /**
     * @brief Write IMU data to the slave.
     * @param accel Accelerometer data to write.
     * @param euler Euler angles to write.
     * @return True if successful, false otherwise.
     */
    bool writeImu(const ImuAccel &accel, const ImuEuler &euler);
    ///@}

    /** @name Encoder operations */
    ///@{
    /**
     * @brief Read encoder angle from the slave.
     * @param outAngle Reference to store the encoder angle.
     * @return True if successful, false otherwise.
     */
    bool readEncoder(double &outAngle);

    /**
     * @brief Write encoder angle to the slave.
     * @param angle Encoder angle to write.
     * @return True if successful, false otherwise.
     */
    bool writeEncoder(double angle);
    ///@}

    /** @name Movement operations */
    ///@{
    /**
     * @brief Read motor speed and steering percentage from the slave.
     * @param outMotorSpeed Reference to store motor speed.
     * @param outSteeringPercent Reference to store steering percentage.
     * @return True if successful, false otherwise.
     */
    bool readMovementInfo(double &outMotorSpeed, float &outSteeringPercent);

    /**
     * @brief Write motor speed and steering percentage to the slave.
     * @param motorSpeed Motor speed to write.
     * @param steeringPercent Steering percentage to write.
     * @return True if successful, false otherwise.
     */
    bool writeMovementInfo(double motorSpeed, float steeringPercent);
    ///@}

private:
    int fd_;
    uint8_t slaveAddr_;

    bool writeRegister(uint8_t reg, const uint8_t *data, size_t len);
    bool readRegister(uint8_t reg, uint8_t *data, size_t len);
};
