#pragma once

#include <cstdint>

#include "imu_struct.h"
#include "pico_i2c_mem_addr.h"

/**
 * @brief Pi-side I2C master to communicate with Pico 2.
 */
class I2cMaster
{
public:
    explicit I2cMaster(uint8_t slave_addr);
    ~I2cMaster();

    bool isInitialized() const;

    /** Command operations */
    bool sendCommand(uint8_t command);
    bool readCommand(uint8_t &command);

    /** Status operations */
    bool readStatus(uint8_t &status);
    bool getIsRunning(bool &isRunning);
    bool getImuReady(bool &imuReady);

    /** IMU operations */
    bool readImu(ImuAccel &accel, ImuEuler &euler);

    bool readEncoder(double &angle);

    /** Movement operations */
    bool readMovement(double &motorSpeed, float &steeringPercent);

private:
    int fd_;
    uint8_t slaveAddr_;

    bool writeRegister(uint8_t reg, const uint8_t *data, size_t len);
    bool readRegister(uint8_t reg, uint8_t *data, size_t len);
};
