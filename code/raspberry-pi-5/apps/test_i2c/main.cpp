#include "i2c_master.h"

#include <cstdio>

int main() {
    I2cMaster master(0x39);
    if (!master.isInitialized()) return -1;

    uint8_t cmd = 0;
    if (master.readCommand(cmd)) printf("Command read: 0x%X\n", cmd);

    uint8_t status = 0;
    if (master.readStatus(status)) printf("Status: 0x%X\n", status);

    ImuAccel accel;
    ImuEuler euler;
    if (master.readImu(accel, euler)) {
        printf("Accel: %.2f %.2f %.2f\n", accel.x, accel.y, accel.z);
        printf("Euler: %.2f %.2f %.2f\n", euler.h, euler.r, euler.p);
    }

    double encoderAngle;
    if (master.readEncoder(encoderAngle)) {
        printf("Encoder Angle: %.2f\n", encoderAngle);
    }

    double motor_speed;
    float steering;
    if (master.readMovementInfo(motor_speed, steering)) {
        printf("Motor speed: %.2f, Steering: %.2f\n", motor_speed, steering);
    }

    return 0;
}
