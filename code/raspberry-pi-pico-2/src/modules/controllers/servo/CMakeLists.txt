# NOTE: servo_controller

add_library(servo_controller STATIC servo_controller.cpp servo_controller.h)

target_include_directories(servo_controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(
  servo_controller
  PRIVATE hardware_pwm
  PUBLIC pico_stdlib)
