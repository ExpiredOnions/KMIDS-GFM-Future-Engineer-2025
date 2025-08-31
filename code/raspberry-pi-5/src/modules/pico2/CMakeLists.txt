# NOTE: pico2_module

add_library(pico2_module STATIC pico2_module.cpp pico2_module.h)
target_include_directories(
  pico2_module PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/src/types
                      ${CMAKE_SOURCE_DIR}/src/shared/types)
target_link_libraries(pico2_module PUBLIC i2c_master ring_buffer logger)
