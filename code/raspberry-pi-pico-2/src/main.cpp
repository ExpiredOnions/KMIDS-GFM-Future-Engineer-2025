#include <hardware/gpio.h>
#include <hardware/i2c.h>
#include <pico/stdio.h>

#include "bno085_controller.h"
#include "i2c_slave.h"

const uint I2C0_SDA_PIN = 4;
const uint I2C0_SCL_PIN = 5;

const uint I2C1_SDA_PIN = 2;
const uint I2C1_SCL_PIN = 3;
const uint I2C1_BAUDRATE = 400000;  // 400 kHz
const uint I2C1_SLAVE_ADDR = 0x39;

int main() {
    // Initialize stdio (optional, for debugging via UART)
    stdio_init_all();

    Bno085Controller imu(i2c0, I2C0_SDA_PIN, I2C0_SCL_PIN);

    gpio_init(I2C1_SDA_PIN);
    gpio_init(I2C1_SCL_PIN);
    gpio_set_function(I2C1_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C1_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C1_SDA_PIN);
    gpio_pull_up(I2C1_SCL_PIN);
    i2c_init(i2c1, I2C1_BAUDRATE);

    // Initialize I2C slave on I2C1, address 0x39
    i2c_slave_init(i2c1, I2C1_SLAVE_ADDR, i2c_slave::handler);

    // Initialize I2C slave context
    i2c_slave::context_init();

    if (!imu.begin()) {
        while (1)
            tight_loop_contents();
    }

    imu.enableRotation(8);
    imu.enableAccelerometer(8);

    imu_accel_float_t accel;
    imu_euler_float_t euler;

    double motor_speed = 0.0;
    float steering_percent = 0.0f;

    // Set initial status
    i2c_slave::set_is_running(true);
    i2c_slave::set_is_imu_ready(false);

    while (true) {
            i2c_slave::set_imu_data(&data.accel, &data.euler);
        if (imu.update(accel, euler)) {
            i2c_slave::set_is_imu_ready(true);
        }

        // Simulate movement info
        motor_speed = 0.76789;
        steering_percent = 0.71237f;

        i2c_slave::set_movement_info(motor_speed, steering_percent);
    }

    return 0;
}
