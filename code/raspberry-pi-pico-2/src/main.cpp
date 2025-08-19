#include "bno08x.h"  // your BNO08x class header
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <cmath>
#include <stdio.h>

#define I2C_PORT i2c0
#define SDA_PIN 16
#define SCL_PIN 17

BNO08x imu;

int main() {
    stdio_init_all();

    // Init I2C
    gpio_init(SDA_PIN);
    gpio_init(SCL_PIN);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
    i2c_init(I2C_PORT, 400000);  // 400kHz I2C

    sleep_ms(2000);

    printf("BNO08x Test Start\n");

    // Start IMU
    if (!imu.begin(0x4a, I2C_PORT)) {
        printf("BNO08x not detected. Check wiring.\n");
        while (1)
            tight_loop_contents();
    }

    printf("BNO08x connected!\n");

    // Enable Rotation Vector
    if (!imu.enableRotationVector(10)) {  // 10 ms interval
        printf("Failed to enable rotation vector!\n");
    }

    while (true) {
        if (imu.getSensorEvent()) {
            if (imu.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
                float roll = imu.getRoll() * 180 / M_PI;
                float pitch = imu.getPitch() * 180 / M_PI;
                float yaw = imu.getYaw() * 180 / M_PI;
                printf("Yaw: %.2f°, Pitch: %.2f°, Roll: %.2f°\n", yaw, pitch, roll);
            }
        }
        sleep_ms(20);
    }
}
