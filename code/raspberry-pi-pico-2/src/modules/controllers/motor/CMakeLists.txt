# NOTE: encoder_controller

add_library(encoder_controller STATIC encoder_controller.cpp
                                      encoder_controller.h)

target_include_directories(encoder_controller
                           PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(encoder_controller PUBLIC pico_stdlib)

# NOTE: motor_controller

add_library(motor_controller STATIC motor_controller.cpp motor_controller.h)

target_include_directories(motor_controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(
  motor_controller
  PRIVATE hardware_pwm
  PUBLIC pico_stdlib)

# NOTE: motor_speed_controller

add_library(motor_speed_controller STATIC motor_speed_controller.cpp
                                          motor_speed_controller.h)

target_include_directories(motor_speed_controller
                           PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(motor_speed_controller
                      PUBLIC motor_controller encoder_controller pid_controller)
