cmake_minimum_required(VERSION 3.13)

# pico_sdk_import.cmake is a single file copied from this SDK
include(external/pico-sdk/pico_sdk_init.cmake)

project(gfm_pico_2 LANGUAGES C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_subdirectory(external/BNO08x_Pico_Library)

add_subdirectory(src/utils)
add_subdirectory(src/modules/controllers)
add_subdirectory(src/modules/i2c_slave)

add_executable(gfm_pico_2 src/main.cpp)
target_link_libraries(gfm_pico_2 pico_stdlib bno085_controller
                      motor_speed_controller i2c_slave)

pico_enable_stdio_usb(gfm_pico_2 1)
pico_enable_stdio_uart(gfm_pico_2 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(gfm_pico_2)
