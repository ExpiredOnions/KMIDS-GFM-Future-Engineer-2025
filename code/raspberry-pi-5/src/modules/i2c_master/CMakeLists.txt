# NOTE: i2c_master

find_library(WIRINGPI_LIBRARY wiringPi)
if(WIRINGPI_LIBRARY)
  set(WIRINGPI_FOUND TRUE)
  message(STATUS "wiringPi library found: ${WIRINGPI_LIBRARY}")
else()
  set(WIRINGPI_FOUND FALSE)
  message(
    WARNING
      "wiringPi library not found, skipping executables requiring wiringPi.")
endif()

add_library(i2c_master STATIC i2c_master.cpp i2c_master.h)
target_include_directories(
  i2c_master
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/src/shared/types
         ${CMAKE_SOURCE_DIR}/src/shared/i2c)
target_link_libraries(i2c_master PUBLIC wiringPi)
