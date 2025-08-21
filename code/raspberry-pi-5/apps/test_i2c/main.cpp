#include "i2c_master.h"

#include <cstdio>

int main() {
    I2CMaster master(0x39);
    if (!master.is_initialized()) return -1;

    uint8_t cmd = 0;
    if (master.read_command(cmd)) printf("Command read: 0x%X\n", cmd);

    uint8_t status = 0;
    if (master.read_status(status)) printf("Status: 0x%X\n", status);

    imu_accel_float_t accel;
    imu_euler_float_t euler;
    if (master.read_imu(accel, euler)) {
        printf("Accel: %.2f %.2f %.2f\n", accel.x, accel.y, accel.z);
        printf("Euler: %.2f %.2f %.2f\n", euler.h, euler.r, euler.p);
    }

    double motor_speed;
    float steering;
    if (master.read_movement(motor_speed, steering)) {
        printf("Motor speed: %.2f, Steering: %.2f\n", motor_speed, steering);
    }

    return 0;
}
