#include <hardware/gpio.h>
#include <hardware/i2c.h>
#include <pico/stdio.h>
#include <pico/time.h>

#include "bno085_controller.h"
#include "encoder_controller.h"
#include "i2c_slave.h"
#include "motor_controller.h"
#include "motor_speed_controller.h"

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

    MotorController motor(6, 7);
    motor.begin();

    EncoderController encoder(10, 11, 28, 50);
    encoder.begin();

    MotorSpeedController motorPid(motor, encoder);

    if (!imu.begin()) {
        while (1)
            tight_loop_contents();
    }

    imu.enableRotation(8);
    imu.enableAccelerometer(8);

    ImuAccel accel;
    ImuEuler euler;

    double motorSpeed = 0.0;
    float steeringPercent = 0.0f;

    // Set initial status
    i2c_slave::set_is_running(true);
    i2c_slave::set_is_imu_ready(false);

    auto lastPrint = std::chrono::steady_clock::now();
    const auto printInterval = std::chrono::milliseconds(100);  // print every 0.5s
    while (true) {
        if (imu.update(accel, euler)) {
            i2c_slave::set_imu_data(accel, euler);
            i2c_slave::set_is_imu_ready(true);
        }

        // Simulate movement info
        motorSpeed = 0.76789;
        steeringPercent = 0.71237f;

        motorPid.update();
        motorPid.setTargetRPS(-1.0);

        sleep_ms(1);

        // Only print every 0.5 seconds
        auto now = std::chrono::steady_clock::now();
        if (now - lastPrint >= printInterval) {
            printf("%.4f\n", motorPid.getPower());
            lastPrint = now;
        }
    }

    return 0;
}
