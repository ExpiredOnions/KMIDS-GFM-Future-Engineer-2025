# NOTE: log_reader

add_library(log_reader STATIC log_reader.cpp log_reader.h)
target_include_directories(log_reader PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
