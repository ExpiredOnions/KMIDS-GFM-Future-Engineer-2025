# NOTE: logger

add_library(logger STATIC logger.cpp logger.h)
target_include_directories(logger PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
