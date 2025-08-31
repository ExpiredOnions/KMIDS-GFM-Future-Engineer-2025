add_library(i2c_slave STATIC i2c_slave.cpp i2c_slave.h)

target_include_directories(
  i2c_slave
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/src/shared/types
         ${CMAKE_SOURCE_DIR}/src/shared/i2c)

target_link_libraries(i2c_slave PUBLIC pico_stdlib pico_i2c_slave)
