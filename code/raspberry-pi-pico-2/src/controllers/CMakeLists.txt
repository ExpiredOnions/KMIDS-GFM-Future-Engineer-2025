# NOTE: bno085_controller

add_library(bno085_controller STATIC bno085_controller.cpp bno085_controller.h)

target_include_directories(
  bno085_controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
                           ${CMAKE_SOURCE_DIR}/src/shared/types)

target_link_libraries(
  bno085_controller
  PRIVATE pico_stdlib hardware_i2c
  PUBLIC BNO08x_Pico_Library)

# NOTE: encoder_controller

add_library(encoder_controller STATIC encoder_controller.cpp
                                      encoder_controller.h)

target_include_directories(encoder_controller
                           PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(encoder_controller PRIVATE pico_stdlib hardware_i2c)

# NOTE: motor_controller

add_library(motor_controller STATIC motor_controller.cpp motor_controller.h)

target_include_directories(motor_controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(motor_controller PRIVATE pico_stdlib hardware_pwm)
