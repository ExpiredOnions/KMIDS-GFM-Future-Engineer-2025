#include "bno085_controller.h"

#define I2C_PORT i2c0
#define SDA_PIN 16
#define SCL_PIN 17

int main() {
    stdio_init_all();

    Bno085Controller imu(I2C_PORT, SDA_PIN, SCL_PIN);

    printf("BNO08x Test Start\n");

    if (!imu.begin()) {
        while (1)
            tight_loop_contents();
    }

    imu.enableRotation(8);
    imu.enableAccelerometer(8);

    TimedImuData data;

    while (true) {
        if (imu.update(data)) {
            printf(
                "Euler: H=%.2f° P=%.2f° R=%.2f° | "
                "Accel: X=%.2f Y=%.2f Z=%.2f\n",
                data.euler.h,
                data.euler.p,
                data.euler.r,
                data.accel.x,
                data.accel.y,
                data.accel.z
            );
        }
        sleep_ms(100);
    }
}
