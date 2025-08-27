# NOTE: pico2_processor

add_library(pico2_processor STATIC pico2_processor.cpp pico2_processor.h)
target_include_directories(
  pico2_processor
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/src/types
         ${CMAKE_SOURCE_DIR}/src/shared/types)
target_link_libraries(pico2_processor ring_buffer)
