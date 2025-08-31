import sys

import smbus2

DEVICE_BUS = 1
DEVICE_ADDR = 0x17  # Updated default I2C address


def set_shutdown_countdown(seconds: int):
    """Write shutdown countdown to UPS (0=False, 1-255 seconds)."""
    bus = smbus2.SMBus(DEVICE_BUS)
    if seconds < 0 or seconds > 255:
        raise ValueError("Seconds must be 0-255")
    bus.write_byte_data(DEVICE_ADDR, 0x18, seconds)
    print(f"Shutdown countdown set to {seconds} seconds.")


if __name__ == "__main__":
    # Default 10 seconds if no argument provided
    seconds = 10
    if len(sys.argv) > 1:
        seconds = int(sys.argv[1])
    set_shutdown_countdown(seconds)
