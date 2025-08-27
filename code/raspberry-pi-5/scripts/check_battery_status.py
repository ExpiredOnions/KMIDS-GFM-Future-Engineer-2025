import smbus2

DEVICE_BUS = 1
DEVICE_ADDR = 0x17  # Default I2C address for EP-0136


def read_register_16(reg_low):
    """Read 16-bit value from two consecutive registers (low byte first)."""
    bus = smbus2.SMBus(DEVICE_BUS)
    low = bus.read_byte_data(DEVICE_ADDR, reg_low)
    high = bus.read_byte_data(DEVICE_ADDR, reg_low + 1)
    return (high << 8) | low


def read_battery_voltage():
    voltage_mv = read_register_16(0x05)
    return voltage_mv / 1000  # convert mV to V


def battery_percentage(voltage):
    FULL_VOLTAGE = 4.2
    EMPTY_VOLTAGE = 3.7
    voltage = max(min(voltage, FULL_VOLTAGE), EMPTY_VOLTAGE)
    return round((voltage - EMPTY_VOLTAGE) / (FULL_VOLTAGE - EMPTY_VOLTAGE) * 100, 1)


def read_battery_temperature():
    temp = read_register_16(0x0B)
    # Temperature range is -20 to 65°C mapped to 0-65535?
    # If register is already in °C, just return as is
    return temp


if __name__ == "__main__":
    voltage = read_battery_voltage()
    percent = battery_percentage(voltage)
    temperature = read_battery_temperature()

    print(f"Battery Voltage: {voltage:.2f} V")
    print(f"Battery Percentage: {percent}%")
    print(f"Battery Temperature: {temperature} °C")
