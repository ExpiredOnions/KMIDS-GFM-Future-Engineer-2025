# NOTE: combined_processor

add_library(combined_processor STATIC combined_processor.cpp
                                      combined_processor.h)
target_include_directories(
  combined_processor
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/src/types
         ${CMAKE_SOURCE_DIR}/src/shared/types)
target_link_libraries(combined_processor ring_buffer)
